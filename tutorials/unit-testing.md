# Guide to ROS 2 unit testing

This guide explains some of the aspects of creating and executing units tests in ROS 2 ament packages.


## Adding tests to a package

The following code is an example of how to add some tests to an ament package:

    if(BUILD_TESTING)
      # Include linting tests
      find_package(ament_lint_auto REQUIRED)
      ament_lint_auto_find_test_dependencies()

      # Add a gtest named ${PROJECT_NAME}_my_test
      ament_cmake_gtest(${PROJECT_NAME}_my_test test/my_test.cpp)

      # Add a nose test named ${PROJECT_NAME}_my_nose_tests
      find_package(ament_cmake_nose REQUIRED)
      ament_add_nose_test(${PROJECT_NAME}_my_nose_tests test/my_nose_tests.py)
    endif()

NOTE: BUILD_TESTING replaces CATKIN_ENABLE_TESTING used in catkin packages.

In order to use gtests and nose tests, you must add the following code to your package.xml:

    <test_depend>ament_cmake_gtest</test_depend>
    <test_depend>ament_cmake_nose</test_depend>


## Compiling unit tests

The following command can be used to compile unit tests for all the packages in your ament workspace:

    ament build --build-tests

This will compile tests for all packages, and will stop when it encounters a package with tests that fail to compile.


## Running all tests

The following command command can be used to run unit tests for all packages in your ament workspace:

    ament test

This will run tests and display the status in your console. It will not stop executing tests if it encounters a failure.

Or you can run tests for just a single ament package:

    ament test src/package_name

NOTE: Replace "src/package_name" with the path to the package you wish to test.


## Viewing a summary of test results

After running tests, the following command can be used to view a summary of the test results:

    ament test_results

This will produce output similar to this:

    Summary: 156 tests, 0 errors, 0 failures, 0 skipped

Specific package failures will be listed in the event that they fail:

    build/pluginlib/test_results/pluginlib/pluginlib_utest.gtest.xml: 7 tests, 0 skipped, 0 errors, 2 failures
    build/ros2_console/test_results/ros2_console/ros2_console_tests.gtest.xml: 17 tests, 0 skipped, 0 errors, 1 failures
    build/ros2_time/test_results/ros2_time/ros2_duration_tests.gtest.xml: 2 tests, 0 skipped, 0 errors, 1 failures
    build/ros2_time/test_results/ros2_time/ros2_time_tests.gtest.xml: 2 tests, 0 skipped, 0 errors, 1 failures
    Summary: 156 tests, 0 errors, 5 failures, 0 skipped

The names (e.g., build/pluginlib/test_results/pluginlib/pluginlib_utest.gtest.xml) comes from the name (i.e., the first argument) passed to the ament_add_gtest command. In the case of pluginlib, the name given was ${PACKAGE_NAME}_utest. The ros2_time time package has two calls to ament_add_gtest which results in multiple failure entries.
