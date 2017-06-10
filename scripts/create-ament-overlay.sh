#!/bin/bash
# This script creates a ROS 2 ament overlay workspace, checks out
# all of the extra repositories, and compiles them
ROS2_WS=~/sandbox/ros2_ws
ROS2_OVERLAY_WS=~/sandbox/TEST_ros2_overlay_ws

# Create the overlay, and checkout the extra repos
mkdir -p $ROS2_OVERLAY_WS/src
cd $ROS2_OVERLAY_WS
wget https://raw.githubusercontent.com/bponsler/ros2-support/master/ros2_extra.repos
vcs import src < ros2_extra.repos
rm ros2_extra.repos  # Clean up the file now that we're done with it

# Compile the repos
cd $ROS2_OVERLAY_WS
source $ROS2_WS/install/local_setup.bash
ament build --build-tests --symlink-install
