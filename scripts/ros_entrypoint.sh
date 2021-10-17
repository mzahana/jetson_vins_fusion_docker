#!/bin/bash
set -e

ros_env_setup="/opt/ros/$ROS_DISTRO/setup.bash"
catkin_env_setup="/root/catkin_ws/devel/setup.bash"
echo "sourcing   $ros_env_setup"
source "$ros_env_setup"
echo "sourcing   $catkin_env_setup"
source "$catkin_env_setup"

echo "ROS_ROOT   $ROS_ROOT"
echo "ROS_DISTRO $ROS_DISTRO"

exec "$@"
