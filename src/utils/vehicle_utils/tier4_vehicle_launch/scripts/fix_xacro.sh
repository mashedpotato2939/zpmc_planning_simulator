#!/bin/bash

# Source ROS environment
source /opt/ros/galactic/setup.bash

# Source the current workspace. Allow deployments to override this path.
WORKSPACE_SETUP=${ZPMC_PLANNING_SETUP:-/home/mash/Zpmc_planning/install/setup.bash}
if [ -f "$WORKSPACE_SETUP" ]; then
  source "$WORKSPACE_SETUP"
fi

# Set ROS package path
export ROS_WORKSPACE=/home/mash/Zpmc_planning
export ROS_PACKAGE_PATH=/home/mash/Zpmc_planning/install:$ROS_PACKAGE_PATH

# Call xacro with all arguments
exec xacro "$@"
