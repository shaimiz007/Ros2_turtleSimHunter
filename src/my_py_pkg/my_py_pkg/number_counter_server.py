
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class PublisherSubsriber_node(Node):
    def __init__(self):
        super().__init__('number_counter')
        self.counter=Int64()
        self.counter.data = 0
        self.pub = self.create_publisher(Int64, 'number_counter', 10)
        self.get_logger().info("Publish on number_conuter started")
        self.sub = self.create_subscription(Int64, 'number', self.callback_news, 10)
      
        
        self.reset_counter_server = self.create_service(SetBool, "reset_number", self.callback_reset_counter)
        self.get_logger().info("number conunter server has been started")
   

    def callback_reset_counter(self, request, response):
        if request.data:
            response.success=True
            response.message="counter has been reset"
            self.counter.data=0
            self.get_logger().info("Reset counter to 0")
        else:
            response.success=False
            response.message="counter has  not been reset"
        return response
      
        
    def callback_news(self, msg):
        self.counter.data+= msg.data
        self.pub.publish(self.counter)
         
def main(args=None):
    rclpy.init(args=args) 
    node_pub_sub = PublisherSubsriber_node()  #  Create instances of NumberCounter
    rclpy.spin(node_pub_sub) # Start spinning the nodes
    rclpy.shutdown()

if __name__ == '__main__':
    main()