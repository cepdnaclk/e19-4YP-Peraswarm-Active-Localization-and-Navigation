#!/bin/bash

# UAV VM IP
export ROS_IP=192.168.208.128

# Assuming this UAV VM is the ROS Master (or update if different)
export ROS_MASTER_URI=http://192.168.208.128:11311

# Source ROS
source /opt/ros/noetic/setup.bash

# Source Catkin workspace
source ~/catkin_ws/devel/setup.bash

# Source PX4
cd ~/PX4-Autopilot


# Set PX4 in ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)

echo "[✅] Environment is now set up for PX4 and ROS."
