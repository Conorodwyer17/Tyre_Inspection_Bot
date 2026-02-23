#!/bin/bash
# Launch autonomous tire inspection mission.
# Runs pre-flight checklist, then starts full stack.
#
# Usage:
#   ./scripts/mission_launch.sh
#   ./scripts/mission_launch.sh ip_address:=192.168.11.1
#   ./scripts/mission_launch.sh dry_run:=true   # validate without motion

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UGV_WS="${UGV_WS:-$(cd "$SCRIPT_DIR/.." && pwd)}"

# Optional: run pre-flight checklist (skip for dry_run or SKIP_PREFLIGHT=1)
if [[ "${SKIP_PREFLIGHT}" != "1" ]] && [[ "$*" != *"dry_run:=true"* ]]; then
  echo "=== Pre-flight checklist ==="
  if bash "$SCRIPT_DIR/aurora_pre_mission_checklist.sh" 2>/dev/null; then
    echo "Pre-flight OK."
  else
    echo "Pre-flight had failures. Continue? [y/N]"
    read -r r
    [[ "$r" =~ ^[yY] ]] || exit 1
  fi
  echo ""
fi

exec bash "$SCRIPT_DIR/startup.sh" "$@"
