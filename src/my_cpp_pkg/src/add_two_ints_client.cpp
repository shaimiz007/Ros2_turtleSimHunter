#include "rclcpp/rclcpp.hpp"                       // Include the RCLCPP library
#include "example_interfaces/srv/add_two_ints.hpp" // Include the AddTwoInts service message

class AddTwoIntsNode : public rclcpp::Node
{
public:
    AddTwoIntsNode() : Node("add_two_ints")
    {
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints"); // Create a client for the AddTwoInts service
        while (!client->wait_for_service(std::chrono::seconds(1)))                              // Wait for the service to be available
        {
            if (!rclcpp::ok()) // Check if ROS 2 is still running
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        for (int i = 1; i <= 5; i++) // Send five requests
        {
            auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>(); // Create a request object
            request->a = 3 + i;                                                              // Set the request values
            request->b = 4 - 2 * i;

            auto future = client->async_send_request(request); // Send the request asynchronously

            // Process callbacks until the response is received or an error occurs
            while (rclcpp::ok() && rclcpp::spin_some(this) != rclcpp::executor::FutureReturnCode::SUCCESS)
            {
                // Wait for the response
            }

            if (future.is_ready()) // Check if the future has a response
            {
                auto response = future.get();                               // Get the response
                RCLCPP_INFO(this->get_logger(), "Sum: %ld", response->sum); // Print the result
            }
            else // if the request fails
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to call service");
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                           // Initialize the ROS 2 communication
    auto node = std::make_shared<AddTwoIntsNode>();      // Create an instance of AddTwoIntsNode
    rclcpp::spin(node);
    rclcpp::shutdown();                                  // Shutdown the ROS 2 communication
    return 0;
}
