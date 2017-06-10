# Accessing ROS parameters

The following code demonstrates how to access a ROS 2 parameter using the SyncParameterClient class:

    #include "rclcpp/node.hpp"
    #include "rclcpp/parameter_client.hpp"

    int main(int argc, char** argv)
    {
        rclcpp::init(argc, argv);
        rclcpp::node::Node::SharedPtr node = rclcpp::node::Node::make_shared("test_node");

        rclcpp::parameter_client::SyncParameterClient client(node);

        double dblVal = client.get_parameter("my_double_val");
        int value = client.get_parameter("my_value", 1);
        if (client.has_parameter("optional_param")) {
            std::string opt = client.get_parameter("optional_param", std::string(""));
        }
    }

First, you must initialize ROS and create a node handle:

    rclcpp::init(argc, argv);
    rclcpp::node::Node::SharedPtr node = rclcpp::node::Node::make_shared("test_node");

Then, using the node, you can create a SyncParameterClient:

    rclcpp::parameter_client::SyncParameterClient client(node);

Now you can get the values of specific parameters:

    double dblVal = client.get_parameter("my_double_val");

This line attempts to get the value of a parameter named "my_double_val". A runtime error is thrown in the event that the parameter does not exist, otherwise the value is stored in the 'dblValue' variable.

    int value = client.get_parameter("my_value", 1);

This will get the value of a parameter named "my_value" and store it in the variable 'value'. If the parameter does not exist, the default value (1) will be returned.

    if (client.has_parameter("optional_param")) {
        std::string opt = client.get_parameter("optional_param", std::string(""));
    }

This code first checks that the parameter exists, and then grabs the value of the "optional_param" value.
