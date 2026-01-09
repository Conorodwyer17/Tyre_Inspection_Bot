#!/bin/bash
# Wrapper script to ensure workspace is sourced before launching mission

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Source the workspace setup
source "$SCRIPT_DIR/install/setup.bash"

# Launch the mission
ros2 launch tyre_inspection_mission autonomous_inspection.launch.py "$@"