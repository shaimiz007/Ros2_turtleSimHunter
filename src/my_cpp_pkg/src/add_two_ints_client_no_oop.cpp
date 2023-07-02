#include "rclcpp/rclcpp.hpp"                       // Include the RCLCPP library
#include "example_interfaces/srv/add_two_ints.hpp" // Include the AddTwoInts service message

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("add_two_ints_client");
    auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints"); // Create a client for the AddTwoInts service
    while (!client->wait_for_service(std::chrono::seconds(1)))                              // Wait for the service to be available
    {
        if (!rclcpp::ok()) // Check if ROS 2 is still running
        {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 0; // Return without continuing execution
        }
        RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
    }
    for (int i = 1; i <= 5; i++)
    {
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>(); // Create a request object
    request->a = 3+i;                                                                  // Set the request values
    request->b = 4-2*i;

    auto future = client->async_send_request(request); // Send the request asynchronously

    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) // Wait for the response
    {
        auto response = future.get();                               // Get the response
        RCLCPP_INFO(node->get_logger(), "Sum: %ld", response->sum); // Print the result
    }
    else            //if the request fails
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service");
    }


    }
    
    rclcpp::shutdown();
    return 0;
}
