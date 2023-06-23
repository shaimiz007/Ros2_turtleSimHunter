#include "rclcpp/rclcpp.hpp"  // Include the RCLCPP library

#include "example_interfaces/srv/add_two_ints.hpp"  // Include the AddTwoInts service message

using std::placeholders::_1;
using std::placeholders::_2;

class AddTwoIntsServerNode : public rclcpp::Node  // Define a class named AddTwoIntsServer that extends rclcpp::Node
{
public:
    AddTwoIntsServerNode() : Node("add_two_ints_server")  // Constructor for AddTwoIntsServer
    {
        server_ = this->create_service<example_interfaces::srv::AddTwoInts>(  // Create a service named "add_two_ints"
            "add_two_ints",
            std::bind(&AddTwoIntsServerNode::callbackAddTwoInts, this, _1, _2));  // Bind the callbackAddTwoInts member function
        RCLCPP_INFO(this->get_logger(), "Service server has been started.");  // Log a message
    }

private:
    void callbackAddTwoInts(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                           const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
    {
        response->sum = request->a + request->b;  // Calculate the sum of the two integers
        RCLCPP_INFO(this->get_logger(), "%ld + %ld = %ld", request->a, request->b, response->sum);  // Log the result
    }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;  // Declare a service member variable
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);  // Initialize the ROS 2 node
    auto node = std::make_shared<AddTwoIntsServerNode>();  // Create an instance of AddTwoIntsServer
    rclcpp::spin(node);  // Run the node
    rclcpp::shutdown();  // Shutdown the ROS 2 node
    return 0;  // Return 0 to indicate successful execution
}
