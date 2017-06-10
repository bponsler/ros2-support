# Writing a ROS 2 launch file
This tutorial will walk through the process of creating a simple file that can be used with the ROS 2 launch system.

The launch code can be found on github at: https://github.com/ros2/launch

## What has changed from ROS (1)?

ROS 2 no longer uses the ROS (1) launch file format (i.e., XML files), instead it makes use of python scripts which provides increased flexibility for configuring nodes.

The launch exectuable has also changed names from ROS (1). It is now simple called *launch* (rather than its previous name of *roslaunch*).

The *launch* executable is not currently capable of locating launch scripts from within a given package. For instance, you used to be able to do this:

    roslaunch my_package my_launch_file.launch

And roslaunch would locate the my_launch_file.launch file contained within the my_package package. This syntax is not currently supported, instead you must pass the full path to the launch script you wish to run.

This tutorial will attempt to explain some minor detail about this format. Support for launch is in a very initial state right now and this tutorial will be updated to contain more information as launch support improves.

## How do I use a ROS 2 launch file?

You can use the following commands (adjust the path to find your ros2 installation directory) to use the ros2 launch program:

    source ~/sandbox/ros2_ws/install/local_setup.bash
    launch --help

This will print the following information:

    $ launch --help
    usage: launch [-h] launch_file [launch_file ...]

    Launch the processes specified in a launch file.

    positional arguments:
      launch_file  The launch file.

    optional arguments:
      -h, --help   show this help message and exit

The *launch* program can take one or more launch files (python scripts) and execute them.

**NOTE**: Make sure you pass the full path to the launch scripts as you cannot currently pass a package name to locate the script

## How do I write a launch script?

Each roslaunch script must be a python script, and contain a "launch" function that will be executed as the file gets launched. For example:

    def launch(descriptor, argv):
        descriptor.add_process(["my_executable"])

The first argument is a *LaunchDescriptor* object which provides the following functions:

- add_coroutine(coroutine, name=None, exit_handler=None)
- add_process(cmd, name=None, env=None, output_handlers=None, exit_handlers=None)

These functions allow you to add a coroutine and a process, respectively, which will be launched as part of the launch tree.

The *add_coroutine* functions allows you to add an asyncio coroutine.

The *add_process* function takes a command as its first argument which should be a list of strings like you would pass to subprocess.call. For example, to execute the following command:

    my_example_node --live-forever --debug=10 /etc/my_config_file.txt

You would pass the following python list as the first value:

    ["my_example_node", "--list-forever", "--debug=10", "/etc/my_config_file.txt"]

It's the same as if you took the command and split it by spaces (e.g., cmd.split(" ")).

The output_handlers and exit_handlers are optional values to the add_process function which default to printing output to the console, and a default exit handler provided by the launch utility. You can override these values as you see fit.

Provided exit handlers include:
- default_exit_handler: trigger a teardown of the launch tree
- ignore_exit_handler: ignores the fact that the process died
- restart_exit_handler: restart the node
- exit_on_error_exit_handler: exit if the return code of the process indicates an error (i.e., is no zero). If it is zero, continue with other tasks
- ignore_signal_exit_handler: succeeds if the process received a shutdown signal
- primary_ignore_returncode_exit_handler: trigger a teardown of the launch tree and ignore return codes

The exit handlers can be imported as follows:

    from launch.exit_handler import default_exit_handler
    from launch.exit_handler import restart_exit_handler

Provided output handlers include:
- FileOutput: process output is written to a file
- ConsoleOutput: process output is written to the console

These handlers can be imported as follows:

    from launch.output_handler import ConsoleOutput
    from launch.output_handler import FileOutput

## Launch parameters

Launch arguments/parameters are not yet supported.
