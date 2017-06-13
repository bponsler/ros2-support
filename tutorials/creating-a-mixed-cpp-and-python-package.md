# How to create a mixed C++ and python ament package

The following shows an example package which contains both C++ code and a python module.

As reference, the example package has the following filesystem layout:

    my_example_package/
	CMakeLists.txt
	package.xml
        include/
	    my_example_package/
	        some_file.hpp
	src/
	    some_file.cpp
	my_example_package/
	    __init__.py
	    some_module.py
	    another_module.py
	scripts/
	    my_executable

First we have the package.xml file:

    <?xml version='1.0' encoding='utf-8'?>
    <package format="2">
      <name>my_example_package</name>
      <version>0.4.8</version>
      <description>examplem mixed C++ and python package</description>

      <maintainer email="dthomas@osrfoundation.org">Dirk Thomas</maintainer>
      <maintainer email="ablasdel@gmail.com">Aaron Blasdel</maintainer>
    
      <license>BSD</license>
    
      <url type="website">http://wiki.ros.org/rqt_dep</url>
      <url type="repository">https://github.com/ros-visualization/rqt_dep</url>
      <url type="bugtracker">https://github.com/ros-visualization/rqt_dep/issues</url>
    
      <author>Thibault Kruse</author>

      <buildtool_depend>ament</buildtool_depend>
      <buildtool_depend>ament_cmake_python</buildtool_depend>

      <depend>rclcpp</exec_depend>
      <depend>rclpy</exec_depend>

      <export>
        <build_type>ament_cmake</build_type>
      </export>
    </package>

This adds a build tool dependency on ament and ament_cmake_python.

Next the CMakeLists.txt:

    cmake_minimum_required(VERSION 2.8.3)
    project(my_example_package)

    # Add support for C++11
    if(NOT WIN32)
        add_definitions(-std=c++11)
    endif()

    find_package(ament_cmake REQUIRED)
    find_package(ament_cmake_python REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rclpy REQUIRED)

    # Install the python module for this package
    ament_python_install_package(${PROJECT_NAME})

    include_directories(include ${rclcpp_INCLUDE_DIRS})
    link_directories(${rclcpp_LIBRARY_DIRS})
    
    # Add a c++ library
    add_library(${PROJECT_NAME} src/some_file.cpp)
    target_link_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})

    # Install C++ headers
    install(
      DIRECTORY include/${PROJECT_NAME}/
      DESTINATION include/${PROJECT_NAME}
      FILES_MATCHING PATTERN "*.hpp")

    # Install python scripts
    install(PROGRAMS scripts/my_executable DESTINATION bin)

    # Export package dependencies
    ament_export_dependencies(ament_cmake)
    ament_export_dependencies(ament_cmake_python)
    ament_export_dependencies(rclcpp)
    ament_export_dependencies(rclpy)
    ament_export_include_directories(include ${rclcpp_INCLUDE_DIRS})
    ament_export_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})

    ament_package()

First, this package will find the necessary dependencies:

    find_package(ament_cmake REQUIRED)
    find_package(ament_cmake_python REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rclpy REQUIRED)

Then, it installs the package as a python module:

    ament_python_install_package(${PROJECT_NAME})

The ${PROJECT_NAME} argument is the name of the folder containing the code for the python module and will also be the name of the installed python module. Typically ROS packages will locate this under the src directory, but passing src/${PROJECT_NAME} as the argument will result in the python module being installed to lib/pythonX.Y/site-packages/src/${PROJECT_NAME} which is not correct. To fix this issue, the folder that contains the python module should be moved out of the src directory and placed directly into the package directory.

This package creates a single C++ executable with the same name as the project:

    add_library(${PROJECT_NAME} src/some_file.cpp)
    target_link_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})

And installs the C++ header files:

    install(
      DIRECTORY include/${PROJECT_NAME}/
      DESTINATION include/${PROJECT_NAME}
      FILES_MATCHING PATTERN "*.hpp")

And finally, installs the single python script:

    install(PROGRAMS scripts/my_executable DESTINATION bin)
