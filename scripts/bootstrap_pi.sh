#!/usr/bin/env bash
#
# bootstrap_pi.sh -- provision a fresh Raspberry Pi 4 into a running tower.
#
# Takes a Pi that has nothing on it but a clean Ubuntu 24.04 Server arm64 image
# and leaves it with ROS 2 Jazzy, the pigpio daemon, a correctly-constructed
# virtualenv, and a built colcon workspace.
#
# Usage:
#     ./scripts/bootstrap_pi.sh                 # full provision on a Pi
#     ./scripts/bootstrap_pi.sh --sim-gpio      # no Pi attached (laptop/CI)
#     ./scripts/bootstrap_pi.sh --skip-build    # system deps and venv only
#     ./scripts/bootstrap_pi.sh --skip-apt      # rebuild without touching apt
#
# The script is idempotent: running it twice is safe and the second run is
# mostly no-ops. That matters, because the way you find out whether a
# provisioning script actually works is by running it again on a second card.
#
# It is deliberately NOT run as root. The venv and the build directory must end
# up owned by the login user, or every subsequent colcon build needs sudo. Only
# the apt and systemctl steps escalate.

set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
ROS_DISTRO_NAME="jazzy"
REQUIRED_UBUNTU="24.04"

# Fallback if GitHub's API is unreachable or rate-limited. The apt source
# package is versioned independently of ROS itself and rarely moves.
ROS_APT_SOURCE_FALLBACK="1.2.0"

# pigpio is built from source; see the pigpio section for why apt cannot
# supply it on Ubuntu. Pinned to a release tag rather than master so two cards
# provisioned a month apart get the same daemon.
PIGPIO_VERSION="v79"
PIGPIO_SRC="/usr/local/src/pigpio"

SIM_GPIO=0
SKIP_BUILD=0
SKIP_APT=0

for arg in "$@"; do
  case "$arg" in
    --sim-gpio)   SIM_GPIO=1 ;;
    --skip-build) SKIP_BUILD=1 ;;
    --skip-apt)   SKIP_APT=1 ;;
    -h|--help)    sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "unknown argument: $arg (try --help)" >&2; exit 2 ;;
  esac
done

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${REPO_ROOT}/venv"

# ---------------------------------------------------------------------------
# Output helpers
# ---------------------------------------------------------------------------
if [ -t 1 ]; then
  BOLD=$'\033[1m'; RED=$'\033[31m'; GREEN=$'\033[32m'; YELLOW=$'\033[33m'; RESET=$'\033[0m'
else
  BOLD=""; RED=""; GREEN=""; YELLOW=""; RESET=""
fi

step() { echo; echo "${BOLD}==> $*${RESET}"; }
info() { echo "    $*"; }
warn() { echo "${YELLOW}    warning: $*${RESET}" >&2; }
die()  { echo "${RED}error: $*${RESET}" >&2; exit 1; }

# ---------------------------------------------------------------------------
# Preflight
# ---------------------------------------------------------------------------
step "Checking the environment"

[ "$(id -u)" -ne 0 ] || die "do not run this as root; it needs to create a venv owned by your login user"
command -v sudo >/dev/null || die "sudo is required"

if [ ! -r /etc/os-release ]; then
  die "cannot read /etc/os-release; this script targets Ubuntu ${REQUIRED_UBUNTU}"
fi

# shellcheck disable=SC1091
. /etc/os-release

ARCH="$(dpkg --print-architecture)"
info "distribution : ${NAME:-unknown} ${VERSION_ID:-unknown} (${VERSION_CODENAME:-unknown})"
info "architecture : ${ARCH}"

if [ "${ID:-}" != "ubuntu" ] || [ "${VERSION_ID:-}" != "${REQUIRED_UBUNTU}" ]; then
  # Not fatal. ROS 2 Jazzy targets Noble specifically, but refusing outright
  # makes the script useless on a dev laptop, which is where --sim-gpio is for.
  warn "expected Ubuntu ${REQUIRED_UBUNTU}; ROS 2 ${ROS_DISTRO_NAME} has no packages for this release"
  warn "continuing anyway -- the apt step will fail loudly if it cannot resolve"
fi

if [ "$ARCH" != "arm64" ] && [ "$SIM_GPIO" -eq 0 ]; then
  warn "architecture is ${ARCH}, not arm64: this is not a Raspberry Pi"
  warn "you probably want --sim-gpio, which stubs pigpio instead of linking it"
fi

# ---------------------------------------------------------------------------
# APT: ROS 2 repository and system packages
# ---------------------------------------------------------------------------
if [ "$SKIP_APT" -eq 1 ]; then
  step "Skipping apt (--skip-apt)"
else
  step "Enabling the universe repository"
  # pigpio lives in universe. On a Server image it is usually already enabled;
  # add-apt-repository is a no-op when it is.
  sudo apt-get update -qq
  sudo apt-get install -y -qq software-properties-common curl ca-certificates
  sudo add-apt-repository -y universe >/dev/null

  step "Configuring the ROS 2 apt repository"
  if [ -f /etc/apt/sources.list.d/ros2.list ] || [ -f /etc/apt/sources.list.d/ros2.sources ]; then
    info "already configured, leaving it alone"
  else
    # The ros2-apt-source package replaced the hand-managed
    # ros-archive-keyring.gpg approach. It carries the signing key and the
    # sources entry together, so a key rotation is an apt upgrade rather than a
    # fleet-wide breakage. Every guide predating mid-2025 still shows the old
    # method; it stopped working when the original key expired.
    APT_SOURCE_VERSION="$(
      curl -fsSL --max-time 15 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest 2>/dev/null \
        | grep -F '"tag_name"' | awk -F'"' '{print $4}'
    )" || true

    if [ -z "${APT_SOURCE_VERSION:-}" ]; then
      warn "could not reach the GitHub API; falling back to ${ROS_APT_SOURCE_FALLBACK}"
      APT_SOURCE_VERSION="$ROS_APT_SOURCE_FALLBACK"
    fi

    DEB="/tmp/ros2-apt-source.deb"
    URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${APT_SOURCE_VERSION}/ros2-apt-source_${APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
    info "installing ros2-apt-source ${APT_SOURCE_VERSION} for ${VERSION_CODENAME}"
    curl -fsSL --max-time 60 -o "$DEB" "$URL" \
      || die "failed to download ${URL}"
    sudo apt-get install -y -qq "$DEB"
    rm -f "$DEB"
  fi

  step "Installing system packages"
  sudo apt-get update -qq

  # Mirrors the CI job's dependency list on purpose. When these two drift, CI
  # stops predicting whether the robot will build, which is the only reason
  # the CI job exists.
  PACKAGES=(
    "ros-${ROS_DISTRO_NAME}-ros-base"
    "ros-${ROS_DISTRO_NAME}-behaviortree-cpp"
    python3-colcon-common-extensions
    python3-venv
    python3-pip
    build-essential
    cmake
    git

    # opencv-python links libGL and libglib, which a Server image has no
    # reason to ship. Without these, `import cv2` dies with
    # "ImportError: libGL.so.1: cannot open shared object file".
    #
    # The tempting alternative -- opencv-python-headless -- does not work
    # here: ultralytics declares a hard dependency on opencv-python, so pip
    # reinstalls the GUI wheel over the headless one on the next
    # `pip install -r requirements.txt` and the fix silently unwinds. Two
    # small system libraries are cheaper than fighting that every upgrade.
    libgl1
    libglib2.0-0t64
  )

  sudo apt-get install -y "${PACKAGES[@]}"
fi

# ---------------------------------------------------------------------------
# pigpio
# ---------------------------------------------------------------------------
if [ "$SIM_GPIO" -eq 1 ]; then
  step "Skipping pigpio (--sim-gpio)"
else
  step "Installing pigpio"
  # Ubuntu 24.04 cannot supply a working pigpio, and this is not a matter of
  # finding the right package name:
  #
  #   - there is no `pigpiod` binary package on noble, on any architecture.
  #     Debian dropped the daemon; upstream pigpio is unmaintained and does
  #     not work on the Pi 5's RP1 at all.
  #   - `libpigpiod-if-dev` does exist, but it is broken: its
  #     /usr/include/pigpiod_if2.h does `#include <pigpio.h>`, and no noble
  #     package ships pigpio.h. Compiling against it fails with
  #     "fatal error: pigpio.h: No such file or directory".
  #
  # Every Raspberry Pi guide says `apt install pigpio`, and every one of them
  # assumes Raspberry Pi OS. On Ubuntu the only route is a source build, which
  # installs headers, both client libraries and the daemon under /usr/local --
  # already on the default include and linker search paths.
  if command -v pigpiod >/dev/null 2>&1 && [ -f /usr/local/include/pigpiod_if2.h ]; then
    info "pigpio already installed ($(command -v pigpiod))"
  else
    info "building pigpio ${PIGPIO_VERSION} from source (a few minutes on a Pi)"
    sudo rm -rf "$PIGPIO_SRC"
    sudo git clone --quiet --depth 1 --branch "$PIGPIO_VERSION" \
      https://github.com/joan2937/pigpio.git "$PIGPIO_SRC"
    sudo make -C "$PIGPIO_SRC" -j"$(nproc)"
    # `make install` also runs `python3 setup.py install`, which is deprecated
    # on 3.12 and emits a wall of setuptools warnings. It still succeeds, and
    # the C artifacts are installed before that step regardless.
    sudo make -C "$PIGPIO_SRC" install
    sudo ldconfig
  fi

  for f in /usr/local/include/pigpiod_if2.h \
           /usr/local/lib/libpigpiod_if2.so \
           /usr/local/bin/pigpiod; do
    [ -e "$f" ] || die "pigpio install incomplete: ${f} is missing"
  done
  info "headers, client library and daemon all present"

  step "Enabling the pigpio daemon"
  # Hardware PWM needs the daemon running before the tracker starts. Turret's
  # constructor calls pigpio_start() and throws if it cannot connect.
  #
  # A source build installs no unit file, so ship our own.
  #
  # `systemctl cat` rather than `systemctl list-unit-files | grep -q`: the
  # latter is a pipeline, and `grep -q` exits at the first match, which
  # SIGPIPEs systemctl mid-write. Under `set -o pipefail` the pipeline then
  # reports 141 and a successful match reads as a failure -- which is exactly
  # how this script came to insist a running pigpiod did not exist.
  if ! systemctl cat pigpiod.service >/dev/null 2>&1; then
    info "installing pigpiod.service"
    sudo install -m 0644 "${REPO_ROOT}/deploy/systemd/pigpiod.service" \
      /etc/systemd/system/pigpiod.service
    sudo systemctl daemon-reload
  fi

  sudo systemctl enable --now pigpiod
  if systemctl is-active --quiet pigpiod; then
    info "pigpiod is running"
  else
    warn "pigpiod is enabled but not active; check: systemctl status pigpiod"
  fi
fi

# ---------------------------------------------------------------------------
# Python virtualenv
# ---------------------------------------------------------------------------
step "Creating the Python virtualenv"

# --system-site-packages is mandatory, not stylistic. rclpy and the generated
# two_towers.msg modules are installed by apt into /opt/ros/jazzy and have no
# PyPI equivalent. An isolated venv cannot see them and the detector dies on
# `import rclpy` before it ever loads a model. See requirements.txt.
if [ -d "$VENV_DIR" ]; then
  if [ -f "${VENV_DIR}/pyvenv.cfg" ] && grep -q 'include-system-site-packages *= *true' "${VENV_DIR}/pyvenv.cfg"; then
    info "reusing existing venv at ${VENV_DIR}"
  else
    warn "existing venv at ${VENV_DIR} was built WITHOUT --system-site-packages"
    warn "it cannot import rclpy; recreating it"
    rm -rf "$VENV_DIR"
    python3 -m venv --system-site-packages "$VENV_DIR"
  fi
else
  python3 -m venv --system-site-packages "$VENV_DIR"
fi

info "installing requirements (torch is large; this takes a while on a Pi)"
"${VENV_DIR}/bin/pip" install --quiet --upgrade pip
"${VENV_DIR}/bin/pip" install --quiet -r "${REPO_ROOT}/requirements.txt"

# Prove the thing this whole venv strategy exists to make work. If ROS is not
# sourced yet in this shell, rclpy will not be importable regardless of the
# venv, so source it first and check for real.
step "Verifying the Python environment"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
[ -f "$ROS_SETUP" ] \
  || die "no ROS 2 ${ROS_DISTRO_NAME} at ${ROS_SETUP} -- re-run without --skip-apt"

# ROS's generated setup scripts reference unset variables, so -u comes off
# across the sourcing and goes straight back on.
set +u
# shellcheck disable=SC1090
. "$ROS_SETUP"
set -u

if "${VENV_DIR}/bin/python" -c 'import rclpy, cv2, flask, ultralytics' 2>/dev/null; then
  info "${GREEN}rclpy, cv2, flask and ultralytics all import${RESET}"
else
  warn "one of rclpy/cv2/flask/ultralytics failed to import; details:"
  "${VENV_DIR}/bin/python" -c 'import rclpy, cv2, flask, ultralytics' || true
  die "the Python environment is not usable; fix the above before continuing"
fi

# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
if [ "$SKIP_BUILD" -eq 1 ]; then
  step "Skipping colcon build (--skip-build)"
else
  step "Building the workspace"
  CMAKE_ARGS=(-DCMAKE_BUILD_TYPE=Release)
  if [ "$SIM_GPIO" -eq 1 ]; then
    CMAKE_ARGS+=(-DTWO_TOWERS_SIM_GPIO=ON)
    info "building with stubbed GPIO; servo commands will be no-ops"
  fi

  cd "$REPO_ROOT"
  colcon build --event-handlers console_direct+ --cmake-args "${CMAKE_ARGS[@]}"
fi

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
step "${GREEN}Bootstrap complete${RESET}"
cat <<EOF

Start a tower by hand:

    source /opt/ros/${ROS_DISTRO_NAME}/setup.bash
    source ${REPO_ROOT}/install/setup.bash
    source ${VENV_DIR}/bin/activate
    ros2 launch two_towers tower.launch.py tower_id:=tower_a

The MJPEG debug stream is then on http://$(hostname).local:5000

To run it as a service instead of by hand, see scripts/install_systemd.sh.
EOF
