#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"

class HardwareStatusPublisher : public rclcpp::Node
{
public:
    HardwareStatusPublisher() : Node("Hardware_status")
    {
        publisher_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                         std::bind(&HardwareStatusPublisher::publishNews, this));
        RCLCPP_INFO(this->get_logger(), "Hardware status started");
    }

private:
    void publishNews()
    {

        auto msg = my_robot_interfaces::msg::HardwareStatus();
        msg.temperature =  1;
        msg.are_motors_ready=true;
        msg.debug_message=std::string("blablabla");
        publisher_->publish(msg);
    }

    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}