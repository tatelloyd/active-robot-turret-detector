#!/usr/bin/env bash
#
# run_tower.sh -- launch one tower with the environment it needs.
#
# This is what the systemd unit executes, but it is a normal script and running
# it by hand is the supported way to debug a tower that will not start:
#
#     TOWER_ID=tower_a TWO_TOWERS_WORKSPACE=~/two-towers \
#     TWO_TOWERS_VENV=~/two-towers/venv ./scripts/run_tower.sh
#
# Everything it needs comes from the environment, which under systemd means
# /etc/two-towers/tower.env plus the per-tower override. See
# deploy/systemd/two-towers@.service.

set -euo pipefail

: "${TOWER_ID:?TOWER_ID is required (systemd supplies it from the unit instance name)}"
: "${TWO_TOWERS_WORKSPACE:?TWO_TOWERS_WORKSPACE is required (set it in /etc/two-towers/tower.env)}"

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
WS_SETUP="${TWO_TOWERS_WORKSPACE}/install/setup.bash"

[ -f "$ROS_SETUP" ] || { echo "no ROS 2 at ${ROS_SETUP}; run scripts/bootstrap_pi.sh" >&2; exit 1; }
[ -f "$WS_SETUP" ]  || { echo "workspace not built: ${WS_SETUP} missing; run colcon build" >&2; exit 1; }

# ROS's generated setup scripts reference unset variables (AMENT_TRACE_SETUP_FILES,
# COLCON_TRACE, and friends), so -u has to come off across the sourcing. It goes
# straight back on afterwards.
set +u
# shellcheck disable=SC1090
. "$ROS_SETUP"
# shellcheck disable=SC1090
. "$WS_SETUP"
set -u

# Put the venv first on PATH rather than activating it. The detector is
# installed as an executable script with a `#!/usr/bin/env python3` shebang, so
# whichever python3 leads PATH is the interpreter it runs under -- that is the
# mechanism by which it gets an environment containing ultralytics. Setting
# VIRTUAL_ENV alongside keeps `pip` and anything else that inspects it honest.
#
# Note this venv was built with --system-site-packages, so /opt/ros's rclpy is
# still visible through it. Both halves are required; see requirements.txt.
if [ -n "${TWO_TOWERS_VENV:-}" ]; then
  [ -x "${TWO_TOWERS_VENV}/bin/python" ] \
    || { echo "TWO_TOWERS_VENV=${TWO_TOWERS_VENV} has no bin/python" >&2; exit 1; }
  export VIRTUAL_ENV="${TWO_TOWERS_VENV}"
  export PATH="${TWO_TOWERS_VENV}/bin:${PATH}"
fi

# Deliberately unquoted: this is a list of `name:=value` launch arguments and
# must word-split. It is set from a systemd EnvironmentFile, which does no
# expansion of its own, so there is nothing here to re-evaluate.
# shellcheck disable=SC2086
exec ros2 launch two_towers tower.launch.py \
  tower_id:="${TOWER_ID}" \
  ${TWO_TOWERS_LAUNCH_ARGS:-}
