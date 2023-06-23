#include "rclcpp/rclcpp.hpp"                       // Include the RCLCPP library
#include "example_interfaces/srv/add_two_ints.hpp" // Include the AddTwoInts service message

class AddTwoIntsClientNode : public rclcpp::Node
{
public:
    AddTwoIntsClientNode() : Node("add_two_ints")
    {
        threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 4, 5)));
    threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 4, 5)));
    }

    void callAddTwoIntsService(int a, int b)
    {
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints"); // Create a client for the AddTwoInts service
        while (!client->wait_for_service(std::chrono::seconds(1)))                              // Wait for the service to be available
        {
            RCLCPP_WARN(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>(); // Create a request object
        request->a = a;                                                                   // Set the request values
        request->b = b;

        auto future = client->async_send_request(request); // Send the request asynchronously

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Sum: %ld", response->sum); // Print the result
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

private:
    
    std::vector<std::thread> threads_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                       // Initialize the ROS 2 communication
    auto node = std::make_shared<AddTwoIntsClientNode>(); // Create an instance of AddTwoIntsClientNode
    rclcpp::spin(node);
    rclcpp::shutdown(); // Shutdown the ROS 2 communication
    return 0;
}