
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
#sys.path.append('~/ros2_ws/install/my_robot_interfaces')

from my_robot_interfaces.msg import HardwareStatus


class HardwareStatusPublisher(Node):
    def __init__(self):
        super().__init__('hardware_publisher')
        self.publisher_ = self.create_publisher(HardwareStatus, 'hardware_status', 10)
        self.timer_=self.create_timer(1,self.publish_news)
 
    def publish_news(self):
        msg=HardwareStatus()
        msg.temperature= 41
        msg.are_motors_ready=True
        msg.debug_message="blablabla"
        self.publisher_.publish(msg)
        self.get_logger().info("Published Hardware_Status")
       


def main(args=None):
    rclpy.init(args=args)

  #  Create instances of NumberPublisher and PubliserSubsriber
    node_num = HardwareStatusPublisher()
    rclpy.spin(node_num)
    # Clean up
    node_num.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
