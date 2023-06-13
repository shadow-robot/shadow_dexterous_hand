#!/bin/bash
# Copyright 2022 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

# PLEASE CHANGE the image value to match the desired Docker Hub Image
image=shadowrobot/dexterous-hand

USAGE="\nA script to output the next available Docker Hub version (e.g. 1.0.29) for a given flavour. Should be run within a git folder. Usage:

next-version.sh [-h] [-d] -f flavour

Arg   Effect
-f    Flavour string, e.g. \"kinetic\"
-d    Enable debug output

Example:
next-version.sh -f kinetic
This would result in \"1.0.29\".\n"

# Set defaults
DEBUG=false

# Get arguments
while getopts "hf:v:d" opt; do
  case $opt in
    h)
      echo -e $USAGE
      exit 0
      ;;
    f)
      TAG_FLAVOUR=$OPTARG
      ;;
    d)
      DEBUG=true
      ;;
    *)
      echo -e $USAGE
      exit 1
      ;;
  esac
done

if [ -z $TAG_FLAVOUR ]; then
  echo -e "Error: No flavour specified. Aborting. Usage instructions:"
  echo -e "$USAGE"
  exit 1
fi

# Get current Docker Hub tags
if $DEBUG ; then echo -e "Checking Docker Hub versions for image:$image:$TAG_FLAVOUR" ; fi
NEW_VERSION="0.0.0"
url=https://hub.docker.com/v2/repositories/$image/tags/?page_size=100
token=$(curl -s -H "Content-Type: application/json" -X POST -d '{"username": "'$DOCKER_HUB_USERNAME'", "password": "'$DOCKER_HUB_PASSWORD'"}' https://hub.docker.com/v2/users/login/ | python -c 'import sys, json; data = json.load(sys.stdin); print(data["token"])')
declare -a ALL_REPO_TAGS=($(
(
  while [ ! $url == "None" ]; do
    content=$(curl -s -H "Authorization: JWT $token" $url | python -c 'import sys, json; data = json.load(sys.stdin); has_next=data.get("next","") if "next" in data else ""; result = " ".join([x["name"] for x in data["results"]]) if "results" in data else ""; print(has_next); print(result)')
    url=$(echo "$content" | head -n 1)
    echo "$content" | tail -n +2
  done;
)  | tr ' ' '\n' | sort -r --version-sort | uniq;))

if [ -z "$ALL_REPO_TAGS" ]; then
  if $DEBUG ; then echo -e "Warning: There are no tags in the Docker Hub for image:$image. The next available tag is:$image:$TAG_FLAVOUR-v$NEW_VERSION" ; fi
  echo $next_version
  exit 0
else
  if $DEBUG ; then echo -e "There are ${#ALL_REPO_TAGS[*]} Docker Hub tags in the repository:" ; fi
  if $DEBUG ; then echo -e ${ALL_REPO_TAGS[@]} ; fi
  HIGHEST_MAJOR=0
  HIGHEST_MINOR=0
  HIGHEST_PATCH=0
  VERSION_REPO_TAGS=()
  VERSION_REPO_TAG_REGEX='^(.*)-v([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3})$'
  for repo_tag in ${ALL_REPO_TAGS[@]}; do
    if [[ "$repo_tag" =~ $VERSION_REPO_TAG_REGEX ]]; then
      if [ "${BASH_REMATCH[1]}" == "$TAG_FLAVOUR" ]; then
        CHECKED_MAJOR=${BASH_REMATCH[2]}
        CHECKED_MINOR=${BASH_REMATCH[3]}
        CHECKED_PATCH=${BASH_REMATCH[4]}
        if [ $CHECKED_MAJOR -gt $HIGHEST_MAJOR ]; then
          HIGHEST_MAJOR=$CHECKED_MAJOR
          HIGHEST_MINOR=$CHECKED_MINOR
          HIGHEST_PATCH=$CHECKED_PATCH
        elif [[ $CHECKED_MAJOR -eq $HIGHEST_MAJOR && $CHECKED_MINOR -gt $HIGHEST_MINOR ]]; then
          HIGHEST_MINOR=$CHECKED_MINOR
          HIGHEST_PATCH=$CHECKED_PATCH
        elif [[ $CHECKED_MAJOR -eq $HIGHEST_MAJOR && $CHECKED_MINOR -eq $HIGHEST_MINOR && $CHECKED_PATCH -gt $HIGHEST_PATCH ]]; then
          HIGHEST_PATCH=$CHECKED_PATCH
        fi
        if $DEBUG ; then echo "HIGHEST_MAJOR=$HIGHEST_MAJOR, HIGHEST_MINOR=$HIGHEST_MINOR, HIGHEST_PATCH=$HIGHEST_PATCH" ; fi
      fi
    fi
  done
  NEW_VERSION="$HIGHEST_MAJOR.$HIGHEST_MINOR.$((HIGHEST_PATCH+1))"
fi
echo $NEW_VERSION
exit 0
