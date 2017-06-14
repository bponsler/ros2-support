#!/bin/bash
# This script checks out the base package repositories and handles
# compiling and installing all of the repositories
BASE_PKG_WS=~/sandbox/base_packages

# Checkout the base package repositories
mkdir -p $BASE_PKG_WS
cd $BASE_PKG_WS
wget https://raw.githubusercontent.com/bponsler/ros2-support/master/base_packages.repos
vcs import . < base_packages.repos
rm base_packages.repos  # Clean up the file now that we're done with it


############################################################
# Install required packages
#
sudo apt-get install -y python3-pyqt5.qtsvg

sudo pip install pydot
sudo pip install numpy

# This fixes an issue with how graphviz gets installed
sudo pip install pygraphviz --install-option="--include-path=/usr/include/graphviz" --install-option="--library-path=/usr/lib/graphviz/"


############################################################
# Compile and install all of the base packages
#
cd $BASE_PKG_WS

# List of base packages that are python packages
PYTHON_PKGS="rosdep"

# Compile and install rosdep
for pkg in $PYTHON_PKGS; do
    pushd $BASE_PKG_WS/$pkg >> /dev/null

    echo "Compiling and installing python base package: $pkg"

    # Install python packages
    python3 setup.py build
    sudo python3 setup.py install
    
    popd >> /dev/null
done
