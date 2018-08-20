#!/bin/bash

USAGE="\nA script to use check whether a git release tag is a valid increment. Should be run within a git folder. Usage:

check-tag.sh [-h] [-d] -f flavour -v version

Arg   Effect
-f    Flavour string, e.g. \"indigo\"
-v    Version string, e.g. \"1.0.1\"
-d    Enable debug output

Example:
check-tag.sh -f kinetic -v 1.1.1
This would result in a tag of \"kinetic-v1.1.1\".\n"

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
    v)
      TAG_VERSION=$OPTARG
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

if [ -z $TAG_VERSION ]; then
  echo -e "Error: No flavour specified. Aborting. Usage instructions:"
  echo -e "$VERSION"
  exit 1
fi

# Check version format and pull out major, minor and patch numbers
VERSION_REGEX='^([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3})$'
if [[ "$TAG_VERSION" =~ $VERSION_REGEX ]]; then
  TAG_MAJOR=${BASH_REMATCH[1]}
  TAG_MINOR=${BASH_REMATCH[2]}
  TAG_PATCH=${BASH_REMATCH[3]}
  TAG_VERSION="${TAG_MAJOR}.${TAG_MINOR}.${TAG_PATCH}"
else
  echo -e "Error: the supplied version string (\"${TAG_VERSION}\") is not a valid semantic version number (\"x.y.z\")."
  exit 1
fi

TAG_FULL="${TAG_FLAVOUR}-v${TAG_VERSION}"

if $DEBUG ; then echo -e "Checking release tag: \"${TAG_FULL}\"" ; fi

# Get current git tags
ALL_REPO_TAGS=($(git tag -l))
if [ $? -ne 0 ]; then
  echo -e "Error: failed to get current Git tags. Aborting."
  exit 1
fi

if [ -z "$ALL_REPO_TAGS" ]; then
  echo -e "Warning: There are no tags in the current git repository. Allowing creation of ${TAG_FULL}."
  exit 0
else # Find current repo tags that are releases, and that are the same flavour as the request
  if $DEBUG ; then echo -e "There are ${#ALL_REPO_TAGS[*]} git tags in the repository:" ; fi
  if $DEBUG ; then echo -e ${ALL_REPO_TAGS[@]} ; fi
  VERSION_REPO_TAGS=()
  VERSION_REPO_VERSIONS=()
  VERSION_REPO_TAGS_FLAVOURS=()
  VERSION_REPO_TAGS_MAJORS=()
  VERSION_REPO_TAGS_MINORS=()
  VERSION_REPO_TAGS_PATCHES=()
  VERSION_REPO_TAG_REGEX='^(.*)-v([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3})$'
  for repo_tag in ${ALL_REPO_TAGS[@]}; do
    if [[ "$repo_tag" =~ $VERSION_REPO_TAG_REGEX ]]; then
      if [ "${BASH_REMATCH[1]}" == "$TAG_FLAVOUR" ]; then
        VERSION_REPO_TAGS+=("$repo_tag")
        VERSION_REPO_VERSIONS+=("${BASH_REMATCH[2]}.${BASH_REMATCH[3]}.${BASH_REMATCH[4]}")
        VERSION_REPO_TAGS_FLAVOURS+=(${BASH_REMATCH[1]})
        VERSION_REPO_TAGS_MAJORS+=(${BASH_REMATCH[2]})
        VERSION_REPO_TAGS_MINORS+=(${BASH_REMATCH[3]})
        VERSION_REPO_TAGS_PATCHES+=(${BASH_REMATCH[4]})
      fi
    fi
  done
  # If there are no previous releases of this flavour, allow any version.
  if [ ${#VERSION_REPO_TAGS[*]} -eq 0 ]; then
    echo -e "Warning: there are no previous versions of the ${TAG_FLAVOUR} flavour. Allowing creation of ${TAG_FULL}."
    exit 0
  fi
  if $DEBUG ; then echo -e "There are ${#VERSION_REPO_TAGS[*]} release tags of the ${TAG_FLAVOUR} release in the repository:" ; fi
  if $DEBUG ; then echo -e ${VERSION_REPO_TAGS[@]} ; fi
fi

# What are the valid previous versions based on the requested tag?
VALID_PREVIOUS_REPO_TAG_REGEXES=()
if [[ $TAG_PATCH -ne 0 ]]; then
  VALID_PREVIOUS_REPO_TAG_REGEXES+=("^${TAG_MAJOR}\.${TAG_MINOR}\.$((${TAG_PATCH}-1))$")
else
  if [[ $TAG_MINOR -ne 0 ]]; then
    VALID_PREVIOUS_REPO_TAG_REGEXES+=("^${TAG_MAJOR}\.$((${TAG_MINOR}-1))\.[0-9]{1,3}$")
  else
    if [[ $TAG_MAJOR -ne 0 ]]; then
      VALID_PREVIOUS_REPO_TAG_REGEXES+=("^$((${TAG_MAJOR}-1))\.[0-9]{1,3}\.[0-9]{1,3}$")
    fi
  fi
fi
if $DEBUG ; then echo -e "Valid previous tag regexes:" ; fi
if $DEBUG ; then echo -e ${VALID_PREVIOUS_REPO_TAG_REGEXES[@]} ; fi

VALID_INCREMENT=0
# Check if valid increment
for i in ${VERSION_REPO_VERSIONS[*]}; do
  # Check if tag already exists
  if [[ "$TAG_VERSION" == "$i" ]]; then
    echo "Error: there is already a ${TAG_VERSION} release! Aborting."
    exit 1
  fi
  # Check if tag is a valid increment
  for regex in ${VALID_PREVIOUS_REPO_TAG_REGEXES[@]}; do
    if [[ "${i}" =~ $regex ]]; then
      if $DEBUG ; then echo -e "${TAG_VERSION} is a valid increment of ${i}." ; fi
      VALID_INCREMENT=1
    fi
  done
done

if [ $VALID_INCREMENT -ne 1 ]; then
  echo -e "Error: ${TAG_VERSION} is not a valid increment of an existing version."
  exit 1
fi
echo -e "We can release ${TAG_FULL}. Proceeding."
exit 0
