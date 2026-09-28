#!/bin/bash
# Source the distribution and the workspace, then run what was asked.
set -e
source /opt/ros/humble/setup.bash
source /eplansys_ws/install/setup.bash
exec "$@"
