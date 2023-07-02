#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class Smartphone : public rclcpp::Node
{
public:
    Smartphone() : Node("Smartphone")
    {
        // Create a subscription to the "robot_news" topic with a queue size of 10
        subscription_ = create_subscription<example_interfaces::msg::String>(
            "robot_news", 10, std::bind(&Smartphone::callback, this, std::placeholders::_1)); //paceholder depand on the # of parameters in callback function
    }

private:
    // Callback function that is called when a message is received
    void callback(const example_interfaces::msg::String::SharedPtr msg)
    {
        // Print the received message data
        RCLCPP_INFO(this->get_logger(),"Recived: %s", msg->data.c_str()); //pmsg - pointer to the instance of example_interfaces::msg::String, data- field inside msg
    }

 // Define the subscription object
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); //initialize the node
    
    auto node = std::make_shared<Smartphone>(); // Create an instance of the MySubscriber class, which is a subclass of rclcpp::Node

    // Spin the node, blocking until SIGINT (Ctrl+C) is received
    rclcpp::spin(node); // Spin the node, blocking until SIGINT (Ctrl+C) is received
      rclcpp::shutdown();   // Shutdown the ROS 2 node
    return 0;
}