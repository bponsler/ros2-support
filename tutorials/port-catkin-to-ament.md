# Porting a ROS (1) catkin package to a ROS 2 ament package

The ros2-support repository includes a script called "catkin-to-ament.py" which
attempts to automate the task of porting a ROS 1 catkin package to use the
ROS 2 ament build system.

This script follows the ROS 2 migration guide found: https://github.com/ros2/ros2/wiki/Migration-Guide

**NOTE: this script is in early development and is LIKELY to have issues**

First check out the necessary source code:

    cd
    git clone https://github.com/bponsler/ros2-support

Next, perform a dry run (this will not change any files):

    cd /path/to/your/package
    python ~/ros2-support/scripts/catkin-to-ament.py --dryrun

Now, execute the porting script for real:

    python ~/ros2-support/scripts/catkin-to-ament.py

Attempt to build your ROS 2 package using ament.
