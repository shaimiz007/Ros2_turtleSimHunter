#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberPublisher : public rclcpp::Node
{
public:
    NumberPublisher() : Node("Number_publisher")
    {
        // Set parameter as int64
        this->declare_parameter("number_to_publish", 777);

        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                         std::bind(&NumberPublisher::publishNews, this));
        RCLCPP_INFO(this->get_logger(), "Number started");
    }

private:
    void publishNews()
    {
        // Get parameter as int64
        int64_t retrieved_param_value;
        this->get_parameter("number_to_publish", retrieved_param_value);

        auto msg = example_interfaces::msg::Int64();
        msg.data = retrieved_param_value;
        publisher_->publish(msg);
    }

    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}