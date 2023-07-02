
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberPublisher_node(Node):
    def __init__(self):
        super().__init__('number')
        self.declare_parameter("number_to_publish",5)
        self.declare_parameter('publish_frequency', 11)
        self.publisher_ = self.create_publisher(Int64, 'number', 10)
        self.timer_=self.create_timer(1/self.get_parameter("publish_frequency").value,self.publish_news)
        
    def publish_news(self):
        msg = Int64()
        msg.data = self.get_parameter("number_to_publish").value
        self.publisher_.publish(msg)
        self.get_logger().info('Published number: %d' % msg.data)
       

    
def main(args=None):
    rclpy.init(args=args)

  #  Create instances of NumberPublisher and PubliserSubsriber
    node_num = NumberPublisher_node()
    rclpy.spin(node_num)
    # Clean up
    node_num.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
