#!/bin/bash

USAGE="\n
A script to use docker-compose to run system-level tests on Shadow releases. Usage:\n
system-test.sh [-b build_tag] \n
\n
Arg   Effect\n
-b    Optional string build tag. If not set, tries to find Jenkins' \$BUILD-TAG environment variable. Falls back to random UUID. \n"

# Get arguments
while getopts "hb:" opt; do
  case $opt in
    h)
      echo -e $USAGE
      exit 0
      ;;
    b)
      TEST_BUILD_TAG=$OPTARG
      ;;
    *)
      echo -e $USAGE
      exit 1
      ;;
  esac
done

if [ -z "$TEST_BUILD_TAG" ]; then
  if [ -z "$BUILD_TAG" ]; then
    # bash generate random 32 character alphanumeric string (upper and lowercase) and 
    TEST_BUILD_TAG=$(cat /dev/urandom | tr -dc 'a-z0-9' | fold -w 32 | head -n 1)
  else
    TEST_BUILD_TAG="$BUILD_TAG"
  fi
fi
# Docker silently removes any special characters from -p arg, so to avoid disagreements:
TEST_BUILD_TAG=$(echo $TEST_BUILD_TAG | tr -dc '[:alnum:]\n\r' | tr '[:upper:]' '[:lower:]')


# Function to kill and remove any docker containers that have the build tag
cleanup() {
  echo "Killing docker-compose containers..."
  docker-compose -p $TEST_BUILD_TAG kill
  echo "Deleting docker-compose containers..."
  docker-compose -p $TEST_BUILD_TAG rm -f
  echo "Deleting docker-compose images..."
  docker rmi -f `docker images | awk '$1~/^'$TEST_BUILD_TAG'/ {print $3}'`
  echo "Removing docker-compose networks..."
  docker network rm `docker network ls | awk '$2~/^'$TEST_BUILD_TAG'/ {print $1}'`
}

# Exit function to ensure cleanup
exit_clean() {
  exit_code=$1
  cleanup
  exit $exit_code
}

# Trap any unexpected errors with an error report and the cleanup function
trap 'cleanup ; printf "Tests failed for unexpected reasons..."' HUP INT QUIT PIPE TERM

#Try running the compose, which will run tests etc.
#stop containers
docker-compose -p $TEST_BUILD_TAG stop
#remove containers created by up previously
docker-compose -p $TEST_BUILD_TAG down
#remove images
docker-compose -p $TEST_BUILD_TAG rm -f
#pull latest images
docker-compose -p $TEST_BUILD_TAG pull
#build and up latest images
docker-compose -p $TEST_BUILD_TAG up --build -d
if [ $? -ne 0 ]; then
  echo "Failed to build test image(s)."
  exit_clean 1
fi
echo "Successfully built test image(s)."
docker attach ${TEST_BUILD_TAG}_system-test_1
if [ $? -ne 0 ]; then
  echo "Failed to run docker compose."
  exit_clean 1
fi

# Wait for the test container to exit, and get its exit code
TEST_EXIT_CODE=`docker wait ${TEST_BUILD_TAG}_system-test_1`

if [ -z ${TEST_EXIT_CODE+x} ] || [ -z "$TEST_EXIT_CODE" ]; then
  # TEST_EXIT_CODE is unset or an empty string
  echo "System tests failed! Exit code unknown."
  exit_clean 1
elif [ "$TEST_EXIT_CODE" -ne 0 ]; then
  # TEST_EXIT_CODE is set, but not 0
  echo "System tests failed! Exit code: $TEST_EXIT_CODE"
else
  echo "System tests passed!"
fi

# Tag and push built image-under-test to Docker hub
DOCKER_HUB_TAG="shadowrobot/dexterous-hand:${release_tag_flavour}-v${release_tag_version}"
docker login -u ${DOCKER_HUB_USERNAME} -p ${DOCKER_HUB_PASSWORD}
if [ $? -ne 0 ]; then
  echo "Error: Failed to log in to Docker Hub. Aborting."
  exit_clean 1
fi
docker tag ${TEST_BUILD_TAG}_system-under-test ${DOCKER_HUB_TAG}
if [ $? -ne 0 ]; then
  echo "Error: Failed to tag built Docker image as \"${DOCKER_HUB_TAG}\". Aborting."
  exit_clean 1
fi
docker push  ${DOCKER_HUB_TAG}
if [ $? -ne 0 ]; then
  echo "Error: Failed to push \"${DOCKER_HUB_TAG}\" to Docker Hub. Aborting."
  exit_clean 1
fi

exit_clean 0
