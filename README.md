# ros2-support
Collection of information/scripts pertaining to ROS 2 development


## Tutorials/Guides
Below are various tutorials provided by this repository:

- [Migrating a catkin package to ament](tutorials/port-catkin-to-ament.md)
- [Accessing ROS parameters](tutorials/accessing-ros-parameters.md)
- [Writing a ROS 2 launch file](tutorials/ros2-launch.md)
- [Guide to ROS 2 unit testing](tutorials/unit-testing.md)
- [How to create a mixed C++ and python ament package](creating-a-mixed-cpp-and-python-package.md)


## Scripts
Below are various scripts provided by this repository:

- [Install base packages](scripts/install-base-packages.sh)
- [Install ROS 2](scripts/install-ros2-source.sh)
- [Port a single catkin package to ament](scripts/catkin-to-ament.py)
- [Create an ament overlay package from extra repositories](scripts/create-ament-overlay.sh)


## Base package dependencies
Below is a list of base packages that are necessary:

- [rosdep](https://github.com/bponsler/rosdep/tree/ros2-devel)


## New ROS 2 packages
Below is a list of ROS 2 packages that were created to faciliate the porting of other packages (see next section):

- [ros2_time](https://github.com/bponsler/ros2_time)
- [ros2_console](https://github.com/bponsler/ros2_console)
- [urg_node_msgs](https://github.com/bponsler/urg_node_msgs)
- [sensor_msgs_util](https://github.com/bponsler/sensor_msgs_util)
- [message_filters](https://github.com/bponsler/message_filters)


## Packages ported to ROS 2
Below is a list of packages that have been ported to ROS 2:

- [cmake_modules](https://github.com/bponsler/cmake_modules/tree/ros2-devel)
- [rospack](https://github.com/bponsler/rospack/tree/ros2-devel)
- [rospkg](https://github.com/bponsler/rospkg/tree/ros2-devel)
- [pluginlib](https://github.com/bponsler/pluginlib/tree/ros2-devel)
- [python_qt_binding](https://github.com/bponsler/python_qt_binding/tree/ros2-devel)
- [qt_gui_core](https://github.com/bponsler/qt_gui_core/tree/ros2-devel)
- [rqt](https://github.com/bponsler/rqt/tree/ros2-devel)
- [rqt_graph](https://github.com/bponsler/rqt_graph/tree/ros2-devel)
- [rqt_dep](https://github.com/bponsler/rqt_dep/tree/ros2-devel)
- [diagnostics](https://github.com/bponsler/diagnostics/tree/ros2-devel)
- [laser_proc](https://github.com/bponsler/laser_proc/tree/ros2-devel)
- [phidgets_drivers](https://github.com/bponsler/phidgets_drivers/tree/ros2-devel)
- [urg_c](https://github.com/bponsler/urg_c/tree/ros2-devel)
- [urg_node](https://github.com/bponsler/urg_node/tree/ros2-devel)
- [angles](https://github.com/bponsler/angles/tree/ros2-devel)
- [vision_opencv](https://github.com/bponsler/vision_opencv/tree/ros2-devel)
- [interactive_markers](https://github.com/bponsler/interactive_markers/tree/ros2-devel)
- [laser_geometry](https://github.com/bponsler/laser_geometry/tree/ros2-devel)
- [navigation_msgs](https://github.com/bponsler/navigation_msgs/tree/ros2-devel)
- [resource_retriever](https://github.com/bponsler/resource_retriever/tree/ros2-devel)
- [xacro](https://github.com/bponsler/xacro/tree/ros2-devel)
