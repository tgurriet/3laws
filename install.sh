#!/usr/bin/env bash
SCRIPT_VERSION="2.0.0"

# Exit on errors
set -e

# Colors
NC='\033[0m'          # Text Reset
On_Black='\033[40m'   # Black background
On_Cyan='\033[46m'    # Cyan background
On_Green='\033[42m'   # Green background
BIRed='\033[1;91m'    # Bold Intense Red
BWhite='\033[1;37m'   # Bold White
BIYellow='\033[1;93m' # Bold Intense Yellow
White='\033[0;37m'    # White

# Printing utilities
ColErr=${BIRed}${On_Black}
ColPrompt=${BWhite}${On_Cyan}
ColInfo=${White}
ColWarn=${BIYellow}${On_Black}
cout() {
  echo -e "${ColInfo}${1}${NC}"
}
cin() {
  echo -e "${ColPrompt}${1}${NC}"
}
cerr() {
  echo -e "${ColErr}${1}${NC}"
}
cwarn() {
  echo -e "${ColWarn}${1}${NC}"
}
ctitle() {
  echo -e "${BWhite}${On_Green}${1}${NC}"
}

# Local functions
promptYesNo() {
  if [[ $ALWAYS_YES == 1 ]]; then
    echo 1
    return
  fi

  local REPLY=${2}
  local TXT="[y/n]"

  if [[ "$REPLY" == 1 ]]; then
    TXT="[Y/n]"
  elif [[ "$REPLY" == 0 ]]; then
    TXT="[y/N]"
  else
    REPLY=""
  fi

  while true; do
    local color=$'\033[1m'
    local noColor=$'\033[0m'
    read -r -e -p "$color${1} $TXT?$noColor " yn

    case $yn in
    y | Y | Yes | yes | YES)
      REPLY=1
      break
      ;;
    n | N | No | no | NO)
      REPLY=0
      break
      ;;
    "")
      if [ -n "$REPLY" ]; then
        break
      fi
      ;;
    *) ;;
    esac
  done

  echo "$REPLY"
}

promptChoiceArch() {
  if [[ $ALWAYS_YES == 1 ]]; then
    cerr "Always Yes selected but a multichoices question has been raised"
    cerr "Use -f -r <ROS_DISTRO> -a <ARCH> -v <UBUNTU_VERSION> to force an install"
    exit 1
  fi

  local REPLY=""
  select which in "amd64" "arm64"; do
    case $which in
    "amd64")
      REPLY="amd64"
      break
      ;;
    "arm64")
      REPLY="arm64"
      break
      ;;
    *) ;;
    esac
  done
  echo "$REPLY"
}

promptChoiceUbuntuVersion() {
  if [[ $ALWAYS_YES == 1 ]]; then
    cerr "Always Yes selected but a multichoices question has been raised"
    cerr "Use -f -r <ROS_DISTRO> -a <ARCH> -v <UBUNTU_VERSION> to force an install"
    exit 1
  fi
  local REPLY=""
  select which in "24.04" "22.04" "20.04"; do
    case $which in
    "24.04")
      REPLY="24.04"
      break
      ;;
    "22.04")
      REPLY="22.04"
      break
      ;;
    "20.04")
      REPLY="20.04"
      break
      ;;
    *) ;;
    esac
  done
  echo "$REPLY"
}

NOBLE_ROS=("jazzy")
JAMMY_ROS=("iron" "humble")
FOCAL_ROS=("galactic" "foxy")

promptChoiceROSDistro() {
  if [[ $ALWAYS_YES == 1 ]]; then
    cerr "Always Yes selected but a multichoices question has been raised"
    cerr "Use -f -r <ROS_DISTRO> -a <ARCH> -v <UBUNTU_VERSION> to force an install"
    exit 1
  fi
  local REPLY=""
  if [ -z "$UBUNTU_VERSION" ]; then
    local ros_list=("jazzy" "iron" "humble" "galactic" "foxy")
  fi

  if [[ $UBUNTU_VERSION == "24.04" ]]; then
    local ros_list=("${NOBLE_ROS[@]}")
  fi

  if [[ $UBUNTU_VERSION == "22.04" ]]; then
    local ros_list=("${JAMMY_ROS[@]}")
  fi

  if [[ $UBUNTU_VERSION == "20.04" ]]; then
    local ros_list=("${FOCAL_ROS[@]}")
  fi

  select which in "${ros_list[@]}"; do
    case $which in
    "jazzy")
      REPLY="jazzy"
      break
      ;;
    "iron")
      REPLY="iron"
      break
      ;;
    "humble")
      REPLY="humble"
      break
      ;;
    "galactic")
      REPLY="galactic"
      break
      ;;
    "foxy")
      REPLY="foxy"
      break
      ;;
    *) ;;
    esac
  done
  echo "$REPLY"
}

promptChoiceTagRelease() {
  local SELECTED_TAG=""
  select choice in "${VALID_TAGS[@]}"; do
    if [[ -n "$choice" ]]; then
      SELECTED_TAG="$choice"
      break
    fi
  done
  echo "$SELECTED_TAG"
}

valid_ros_ubuntu_match() {
  if [[ $UBUNTU_VERSION == "24.04" ]]; then
    for value in "${NOBLE_ROS[@]}"; do
      [[ $value == "$QUERY_ROS_DISTRO" ]] && return 0
    done
    return 1
  fi

  if [[ $UBUNTU_VERSION == "22.04" ]]; then
    for value in "${JAMMY_ROS[@]}"; do
      [[ $value == "$QUERY_ROS_DISTRO" ]] && return 0
    done
    return 1
  fi

  if [[ $UBUNTU_VERSION == "20.04" ]]; then
    for value in "${FOCAL_ROS[@]}"; do
      [[ $value == "$QUERY_ROS_DISTRO" ]] && return 0
    done
    return 1
  fi
}

valid_tag_release() {
  MATCHED_TAG=$(printf "%s\n" "${VALID_TAGS[@]}" | grep -x "$WANTED_RELEASE_TAG")

  if [[ -n "$MATCHED_TAG" ]]; then
    return 0
  fi
  return 1
}

# Usage info
show_help() {
  cat <<EOF
Usage: ${0##*/} [-hyf] [-r <ROS_DISTRO>] [-a <ARCH>] [-v <UBUNTU_VERSION>] [-t <RELEASE_TAG>]
Install 3Laws Supervisor
   -h                 show this help menu
   -y                 answer yes to all yes/no questions
   -f                 Force install, specify all arguments
   -r                 Optional: ROS distribution
   -a                 CPU architecture (arm64v8|amd64)
   -v                 Ubuntu version (24.04|22.04|20.04)
   -t                 Optional: release tag
EOF
}

check_values() {
  # No variable should be empty
  if [[ -z $ARCH ]]; then
    cerr "Architecture not found, specify amd64|arm64"
    ARCH=$(promptChoiceArch)
  fi
  if [[ -z $UBUNTU_VERSION ]]; then
    cerr "Ubuntu version not found, specify 24.04|22.04|20.04"
    UBUNTU_VERSION=$(promptChoiceUbuntuVersion)
  fi
  if [[ -z $QUERY_ROS_DISTRO ]]; then
    cerr "ROS distribution not found, specify jazzy|iron|humble|galactic|foxy"
    QUERY_ROS_DISTRO=$(promptChoiceROSDistro)
  fi
  if [[ -z $WANTED_RELEASE_TAG ]]; then
    cerr "Release tag not found, specify a valid tag"
    WANTED_RELEASE_TAG=$(promptChoiceTagRelease)
  fi

  # Check if the specified ROS distribution is compatible with the selected Ubuntu version
  if ! valid_ros_ubuntu_match; then
    cwarn "Specified ROS distribution not compatible with Ubuntu \"$UBUNTU_VERSION\""
    echo "24.04: jazzy"
    echo "22.04: iron | humble"
    echo "20.04: galactic | foxy"
    if [ "$FORCE" == 1 ]; then
      cout "Retry with other arguments"
      exit 1
    fi
    cout "Select option number:"
    QUERY_ROS_DISTRO=$(promptChoiceROSDistro)
  fi

  # Check if the specified release tag is valid
  if ! valid_tag_release; then
    cwarn "Specified release tag is not a valid option."
    cout "Available options are: ${VALID_TAGS[*]}"
    if [ "$FORCE" == 1 ]; then
      cout "Retry with other arguments"
      exit 1
    fi
    cout "Select option number:"
    WANTED_RELEASE_TAG=$(promptChoiceTagRelease)
  fi
}

# Script Options
ALWAYS_YES=0
FORCE=0
WANTED_ROS=""
WANTED_ARCH=""
WANTED_UBUNTU=""
WANTED_RELEASE_TAG=""

# Define GIT variables and check connection
GH_API="https://api.github.com"
GH_REPO="$GH_API/repos/3LawsRobotics/3laws"
curl -o /dev/null -s $GH_REPO || {
  cerr "Error: Invalid repo, token or network issue!"
  exit 1
}

# Create a list of valid tags
VALID_TAGS=()
mapfile -t VALID_TAGS < <(curl -s "$GH_REPO/tags" |
  jq -r '.[].name' |
  sed 's/^supervisor_//')
VALID_TAGS=("latest" "${VALID_TAGS[@]}")

if [ "${#VALID_TAGS[@]}" -eq 0 ]; then
  cerr "No tags found in the repository. Please contact support@3lawsrobotics.com"
  exit 1
fi

while getopts hyfr:a:v:t: opt; do
  case $opt in
  h)
    show_help
    exit 0
    ;;
  y)
    ALWAYS_YES=1
    ;;
  f)
    FORCE=1
    ;;
  r)
    WANTED_ROS="$OPTARG"
    ;;
  a)
    WANTED_ARCH="$OPTARG"
    ;;
  v)
    WANTED_UBUNTU="$OPTARG"
    ;;
  t)
    WANTED_RELEASE_TAG="$OPTARG"
    ;;
  *)
    show_help >&2
    exit 1
    ;;
  esac
done
shift "$((OPTIND - 1))"

# Main
ctitle "3Laws Supervisor Installer (v$SCRIPT_VERSION)"

if [ "$FORCE" == 1 ] && { [ -z "$WANTED_ARCH" ] || [ -z "$WANTED_ROS" ] || [ -z "$WANTED_UBUNTU" ] || [ -z "$WANTED_RELEASE_TAG" ]; }; then
  cerr "The force arg requires all information to be provided, arch, ROS, Ubuntu version, and release tag"
  exit 1
fi

cout "Fetching system information..."
HAS_ROS2=0
if command -v ros2 &>/dev/null; then
  HAS_ROS2=1
  QUERY_ROS_DISTRO=$ROS_DISTRO
  cout "\t Detected existing ROS2 distribution \"$QUERY_ROS_DISTRO\""
fi

UBUNTU_VERSION=$(cat /etc/*-release | grep VERSION_ID | grep -oE "[0-9]{2}.[0-9]{2}")
cout "\t Detected Ubuntu version \"$UBUNTU_VERSION\""

ARCH=amd64
case "$(uname -i)" in
arm* | aarch*)
  ARCH=arm64
  ;;
esac
cout "\t Detected architecture \"$ARCH\""

if [ $FORCE == 0 ]; then

  # If an Ubuntu version is specified and it does not match the detected one, prompt for confirmation
  if [ -n "$WANTED_UBUNTU" ]; then
    if [ "$UBUNTU_VERSION" != "$WANTED_UBUNTU" ]; then
      cwarn "Specified Ubuntu version does not match the detected one. Please confirm your choice by selecting option number:"
      UBUNTU_VERSION=$(promptChoiceUbuntuVersion)
    fi
  fi

  # If an architecture is specified and it does not match the detected one, prompt for confirmation
  if [ -n "$WANTED_ARCH" ]; then
    if [[ $ARCH != "$WANTED_ARCH" ]]; then
      cwarn "Specified Architecture does not match the detected one. Please confirm your choice by selecting option number:"
      ARCH=$(promptChoiceArch)
    fi
  fi

  # If a ROS distribution is specified and it does not match the detected one, prompt for confirmation
  if [ -n "$WANTED_ROS" ]; then
    if [ "$QUERY_ROS_DISTRO" != "$WANTED_ROS" ]; then
      cwarn "Specified ROS distribution does not match the selected one. Please confirm your choice by selecting option number:"
      QUERY_ROS_DISTRO=$(promptChoiceROSDistro)
    fi
  fi

  if [ $HAS_ROS2 == 0 ]; then
    cwarn "Unable to select a ROS distribution, select desired:"
    QUERY_ROS_DISTRO=$(promptChoiceROSDistro)
  fi

  if [ -z "$WANTED_RELEASE_TAG" ]; then
    # Using default "latest" tag
    WANTED_RELEASE_TAG="latest"
  fi

else

  if [[ $UBUNTU_VERSION != "$WANTED_UBUNTU" ]]; then
    cwarn "Specified Ubuntu version does not match the detected one, continuing with $WANTED_UBUNTU"
    UBUNTU_VERSION=$WANTED_UBUNTU
  fi

  if [[ $ARCH != "$WANTED_ARCH" ]]; then
    cwarn "Specified Architecture does not match the detected one, continuing with $WANTED_ARCH"
    ARCH=$WANTED_ARCH
  fi

  if [[ $QUERY_ROS_DISTRO != "$WANTED_ROS" ]]; then
    cwarn "Specified ROS distribution does not match the detected one, continuing with $WANTED_ROS"
    QUERY_ROS_DISTRO=$WANTED_ROS
  fi
fi

# Check Write permission
if [ ! -w . ]; then
  cerr "No write permission in current directory, please run this script in a writable directory"
  exit 1
fi

# Check args validity
check_values

# Confirm package to be downloaded
cout "The Supervisor package with tag \"$WANTED_RELEASE_TAG\" for Ubuntu $UBUNTU_VERSION $ARCH and ROS $QUERY_ROS_DISTRO will be downloaded"

# Assemble the URL based on the selected release tag
if [[ $WANTED_RELEASE_TAG != "latest" ]]; then
  WANTED_RELEASE_TAG="tags/supervisor_$WANTED_RELEASE_TAG"
fi
GH_TAGS="$GH_REPO/releases/$WANTED_RELEASE_TAG"

CURL_ARGS="-LJO#"
PACKAGE_NAME="lll-supervisor-full-${QUERY_ROS_DISTRO}"
REGEX_QUERY="${PACKAGE_NAME}_[0-9]\+\.[0-9]\+\.[0-9]\+-[0-9]\+_$ARCH"

# Read asset tags.
RESPONSE=$(curl -s -H "application/vnd.github+json" $GH_TAGS)
ASSET_NAME=$(echo "$RESPONSE" | grep -o "name.:.\+${REGEX_QUERY}.deb" | cut -d ":" -f2- | cut -d "\"" -f2-)
ASSET_ID=$(echo "$RESPONSE" | grep -C3 "name.:.\+$REGEX_QUERY" | grep -w id | tr : = | tr -cd '[[:alnum:]]=' | cut -d'=' -f2-)

[ "$ASSET_ID" ] || {
  VALID_ASSETS=$(echo "$RESPONSE" | grep -o "name.:.\+lll-supervisor-full-[a-zA-Z]\+_[0-9]\+\.[0-9]\+\.[0-9]\+-[0-9]_[a-zA-Z0-9]\+" | cut -d ":" -f2- | cut -d "\"" -f2-)
  if [ -z "$VALID_ASSETS" ]; then
    cerr "No valid assets are available for your configuration, please contact support@3lawsrobotics.com"
  else
    echo -e "Error: Failed to get asset id, valid packages:\n$VALID_ASSETS"
  fi
  exit 1
}

GH_ASSET="$GH_REPO/releases/assets/$ASSET_ID"

DOWNLOAD=0
if [ -f "$ASSET_NAME" ]; then
  cwarn "$ASSET_NAME already in your directory."
  OVERWRITE=$(promptYesNo "Do you want to overwrite $ASSET_NAME in the current directory ?" 1)
  if [ "$OVERWRITE" -eq 1 ]; then
    cwarn "Removing $ASSET_NAME"
    rm "$ASSET_NAME"
    DOWNLOAD=1
  fi
else
  DOWNLOAD=$(promptYesNo "Do you want to download $ASSET_NAME in the current directory" 1)
fi

if [[ $DOWNLOAD == 1 ]]; then
  echo "Downloading package..." >&2
  curl $CURL_ARGS -s -H 'Accept: application/octet-stream' "$GH_ASSET"
  cout "Package $ASSET_NAME has been downloaded."
else
  cwarn "Package not downloaded."
  exit 0
fi

if [[ -f "$ASSET_NAME" ]]; then
  INSTALL=$(promptYesNo "Do you want to install $ASSET_NAME and its dependency (libstd++13) ?" 1)

  if [ "$INSTALL" == 0 ]; then
    cout "Package downloaded but not installed, if you choose to install manually, be sure to have libstd++13 on your system"
  else
    SUDO=""
    if [[ "$EUID" -ne 0 && $INSTALL == 1 ]]; then
      cwarn "To install the supervisor automatically some commands must be run as sudo"
      SUDO="sudo "
    fi
    # Install dependencies
    STDLIB=libstdc++-13-dev
    STDLIB_INSTALLED=0
    dpkg -l $STDLIB &>/dev/null && STDLIB_INSTALLED=1

    if [[ $STDLIB_INSTALLED == 0 ]]; then
      cout "Installing dependencies..."
      $SUDO apt-get update &>/dev/null
    fi

    if [[ $STDLIB_INSTALLED == 0 ]]; then
      {
        {
          $SUDO apt-get install -y --no-install-recommends $STDLIB &>/dev/null
        } || {
          $SUDO apt-get install -y --no-install-recommends software-properties-common &>/dev/null
          $SUDO add-apt-repository -y "ppa:ubuntu-toolchain-r/test" &>/dev/null
          cwarn "Added 'ppa:ubuntu-toolchain-r/test' to apt sources!"
          $SUDO apt-get install -y --no-install-recommends $STDLIB &>/dev/null
        }
        cwarn "Installed '$STDLIB' on system!"
      } || {
        cerr "Failed to install '$STDLIB' dependency!"
        exit 65
      }
    fi

    # Install package
    cout "Installing package..."
    $SUDO apt install -f ./"$ASSET_NAME" -y --no-install-recommends

    # Create 3laws directory
    cout "Creating 3laws config directory..."
    mkdir -p "$HOME/.3laws/config"

    cout "Removing artifacts..."
    rm "$ASSET_NAME"

    cout "Success installation!"
  fi
else
  cout "Package not found...If you encounter any issues, please contact: support@3lawsrobotics.com"
fi
