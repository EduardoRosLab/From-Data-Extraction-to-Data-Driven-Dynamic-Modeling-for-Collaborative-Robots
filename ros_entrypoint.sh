#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.sh
source /ros2_dev/iiwa_ros2/install/setup.sh
source /usr/share/gazebo/setup.bash
source /ros_app/install/setup.bash
exec "$@"
