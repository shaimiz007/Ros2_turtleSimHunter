
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class NumberPublisher_node(Node):
    def __init__(self):
        super().__init__('number')
        self.declare_parameter("number_to_publish",[15,3,4])
        self.publisher_ = self.create_publisher(String, 'number', 10)
        self.timer_=self.create_timer(1,self.publish_news)
        
    def publish_news(self):
        msg = String()
        msg.data = str(self.get_parameter("number_to_publish").value)
        self.publisher_.publish(msg)
        self.get_logger().info('Published number: ' + msg.data)
       

    
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
    
