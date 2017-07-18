#!/bin/bash
# This script checks out and compiles all the packages necessary
# to run rviz in ROS 2.
RVIZ_OVERLAY_WS=~/sandbox/ros2_rviz_overlay_ws

# Set up the rviz overlay workspace
mkdir -p $RVIZ_OVERLAY_WS/src
cd $RVIZ_OVERLAY_WS

# Check out the source packages
wget https://raw.githubusercontent.com/bponsler/ros2-support/master/rviz_ros2.repos
vcs import src < rviz_ros2.repos

# Compile the packages
ament build
