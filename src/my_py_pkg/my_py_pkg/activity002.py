
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
class NumberPublisher_node(Node):
    def __init__(self):
        super().__init__('number_publisher')
        self.publisher_ = self.create_publisher(Int64, 'number', 10)
        self.publish_news()

    def publish_news(self):
        msg = Int64()
        msg.data = 4
        self.publisher_.publish(msg)
        self.get_logger().info('Published number: %d' % msg.data)


class PublisherSubsriber_node(Node):
    def __init__(self):
        super().__init__('publisher_subscriber')
        self.counter = 0
        self.sub = self.create_subscription(Int64, 'number', self.callback_news, 10)
        self.pub = self.create_publisher(Int64, 'number_counter', 10)
        self.get_logger().info("Publish on number_conter started")


    def callback_news(self, msg):
        self.counter += msg.data
        self.pub.publish(self.counter)

    
def main(args=None):
    rclpy.init(args=args)

  #  Create instances of NumberPublisher and PubliserSubsriber
    node_num = NumberPublisher_node()
    node_pub_sub = PublisherSubsriber_node()

    # Start spinning the nodes
    try:
        rclpy.spin(node_num)
        rclpy.spin(node_pub_sub)
    except KeyboardInterrupt:
        pass

    # Clean up
    node_num.destroy_node()
    node_pub_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
