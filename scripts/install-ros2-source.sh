#!/bin/bash
# Taken from: https://github.com/ros2/ros2/wiki/Linux-Development-Setup

ROS2_WS=~/sandbox/ros2_ws

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANGUAGE=en_US.UTF-8

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'
sudo apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

sudo apt-get update
sudo apt-get install git wget
sudo apt-get install -y build-essential cppcheck cmake libopencv-dev libpoco-dev libpocofoundation9v5 libpocofoundation9v5-dbg python-empy python3-dev python3-empy python3-nose python3-pip python3-setuptools python3-vcstool libtinyxml-dev libeigen3-dev
# dependencies for testing
sudo apt-get install -y clang-format pydocstyle pyflakes python3-coverage python3-mock python3-pep8 uncrustify
sudo pip3 install flake8 flake8-import-order
# dependencies for FastRTPS
sudo apt-get install -y libasio-dev libtinyxml2-dev

sudo apt-get install -y libboost-chrono-dev libboost-date-time-dev libboost-program-options-dev libboost-regex-dev libboost-system-dev libboost-thread-dev

# Checkout the source
mkdir -p $ROS2_WS/src
cd $ROS2_WS
wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
vcs import src < ros2.repos

# Compile
cd $ROS2_WS
./src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install
