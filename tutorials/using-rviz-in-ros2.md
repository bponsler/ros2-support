# Using rviz in ROS 2

The following tutorial will walk you through the necessary steps to get rviz working in ROS 2.

**NOTE**: rviz is not currently fully functional in ROS 2 -- many features are missing completely. Do not expect it to work perfectly and do not be surprised if it crashes or has issues.

## Working plugins

The following is a list of plugins that are working to some degree:

- Axes
- Grid
- Tf
- Laser Scan
- Map
- Robot model
- Path
- Point Stamped
- Polygon
- Range
- Relative Humidity
- Temperature
- Wrench Stamped
- Odometry

## Check out and compile the source packages

Use the following steps to checkout and compile all of the packages necessary to compile rviz:

    # Set up the rviz overlay workspace
    mkdir -p ~/sandbox/ros2_rviz_overlay_ws/src
    cd ~/sandbox/ros2_rviz_overlay_ws

    # Check out the source packages
    wget https://raw.githubusercontent.com/bponsler/ros2-support/master/rviz_ros2.repos
    vcs import src < rviz_ros2.repos

    # Compile the packages
    ament build

## Running rviz in ROS 2

The following commands will allow you to run rviz ROS 2:

    source ~/sandbox/ros2_rviz_overlay_ws/install/local_setup.bash
    ros2 run rviz rviz
