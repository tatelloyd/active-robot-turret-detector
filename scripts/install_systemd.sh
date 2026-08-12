#!/usr/bin/env bash
#
# install_systemd.sh -- install the templated tower unit and its config.
#
# Usage:
#     sudo ./scripts/install_systemd.sh              # infer user and paths
#     sudo ./scripts/install_systemd.sh --user lloyd
#     ./scripts/install_systemd.sh --dry-run         # print, change nothing
#
# Installs:
#     /etc/systemd/system/two-towers@.service
#     /etc/two-towers/tower.env      (only if absent -- never clobbered)
#
# Then enable whichever towers this host runs:
#     sudo systemctl enable --now two-towers@tower_a
#
# Idempotent. Re-running updates the unit and leaves your edited tower.env
# alone; the config is yours once it exists, and silently rewriting it during
# an upgrade is how a deployment loses its tuning.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UNIT_SRC="${REPO_ROOT}/deploy/systemd/two-towers@.service"
ENV_SRC="${REPO_ROOT}/deploy/systemd/tower.env.template"

UNIT_DEST="/etc/systemd/system/two-towers@.service"
ENV_DIR="/etc/two-towers"
ENV_DEST="${ENV_DIR}/tower.env"

DRY_RUN=0
TOWER_USER=""

while [ $# -gt 0 ]; do
  case "$1" in
    --user)    TOWER_USER="${2:?--user needs a username}"; shift 2 ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) sed -n '2,22p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1 (try --help)" >&2; exit 2 ;;
  esac
done

die() { echo "error: $*" >&2; exit 1; }

[ -f "$UNIT_SRC" ] || die "missing ${UNIT_SRC}"
[ -f "$ENV_SRC" ]  || die "missing ${ENV_SRC}"

# Infer the owning user. Under sudo, SUDO_USER is the human who invoked it --
# defaulting to root instead would produce a unit that runs the tower as root
# against a venv root does not own.
if [ -z "$TOWER_USER" ]; then
  TOWER_USER="${SUDO_USER:-$(id -un)}"
fi
[ "$TOWER_USER" != "root" ] || die "refusing to install a unit that runs as root; pass --user"
id -u "$TOWER_USER" >/dev/null 2>&1 || die "no such user: ${TOWER_USER}"
TOWER_GROUP="$(id -gn "$TOWER_USER")"

VENV_DIR="${REPO_ROOT}/venv"

echo "workspace : ${REPO_ROOT}"
echo "venv      : ${VENV_DIR}"
echo "user      : ${TOWER_USER}:${TOWER_GROUP}"
echo

[ -d "$VENV_DIR" ] || echo "warning: ${VENV_DIR} does not exist yet; run scripts/bootstrap_pi.sh first" >&2
[ -f "${REPO_ROOT}/install/setup.bash" ] || echo "warning: workspace is not built yet; run colcon build" >&2

# Substitution is done with a bash replacement rather than sed so that paths
# containing slashes need no escaping.
render() {
  local content
  content="$(cat "$1")"
  content="${content//__WORKSPACE__/$REPO_ROOT}"
  content="${content//__VENV__/$VENV_DIR}"
  content="${content//__TOWER_USER__/$TOWER_USER}"
  content="${content//__TOWER_GROUP__/$TOWER_GROUP}"
  printf '%s\n' "$content"
}

if [ "$DRY_RUN" -eq 1 ]; then
  echo "=== ${UNIT_DEST} ==="; render "$UNIT_SRC"
  echo; echo "=== ${ENV_DEST} ==="; render "$ENV_SRC"
  echo; echo "(dry run: nothing written)"
  exit 0
fi

[ "$(id -u)" -eq 0 ] || die "must run as root to write ${UNIT_DEST} (use sudo, or --dry-run)"

install -d -m 0755 "$ENV_DIR"

render "$UNIT_SRC" > "$UNIT_DEST"
chmod 0644 "$UNIT_DEST"
echo "wrote ${UNIT_DEST}"

if [ -e "$ENV_DEST" ]; then
  echo "kept  ${ENV_DEST} (already exists, not overwritten)"
else
  render "$ENV_SRC" > "$ENV_DEST"
  chmod 0644 "$ENV_DEST"
  echo "wrote ${ENV_DEST}"
fi

chmod +x "${REPO_ROOT}/scripts/run_tower.sh"
systemctl daemon-reload

cat <<EOF

Done. Enable the towers this host runs:

    sudo systemctl enable --now two-towers@tower_a

Watch it come up:

    journalctl -u two-towers@tower_a -f

If it fails, run the same thing by hand to see the error directly:

    sudo -u ${TOWER_USER} env \\
        TOWER_ID=tower_a \\
        TWO_TOWERS_WORKSPACE=${REPO_ROOT} \\
        TWO_TOWERS_VENV=${VENV_DIR} \\
        ${REPO_ROOT}/scripts/run_tower.sh
EOF
