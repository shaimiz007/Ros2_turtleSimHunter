
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from my_robot_interfaces.srv import SetLed

class LedNode(Node):
    def __init__(self):
        super().__init__('LED')
        
        #creating publisher
        self.declare_parameter("leds_state",[False]*3)
        self.led_states_=self.get_parameter("leds_state").value
    
        self.publisher_ = self.create_publisher(String, 'LEDS', 10)
        self.timer_=self.create_timer(0.5,self.publish_news)
        self.get_logger().info('Publishing LED state')
       
        #creating server
        self.server_ = self.create_service(SetLed, "LED_setting", self.callback_set_led)
        self.get_logger().info("Set LED server has been started")
   
       
    def callback_set_led(self, request, response):
        self.led_state_[0]=request.powering_led_1
        response.success=True
        # ANSI escape sequences for text colors
        RESET = '\033[0m'
        RED = '\033[31m'
        GREEN = '\033[32m'
        self.get_logger().info("LED 1 turned " + (GREEN +"ON" + RESET if request.powering_led_1 else "OFF"))
        return response

    def publish_news(self):
        msg = String()
        msg.data = "".join(map(lambda x: " \u2588" if x==1 else " \u25AF", self.led_states_))
        self.publisher_.publish(msg)
        self.get_logger().info('Published number: ' + msg.data)
       
       
    
def main(args=None):
    rclpy.init(args=args)

  #  Create instances of NumberPublisher and PubliserSubsriber
    node_num = LedNode()
    rclpy.spin(node_num)
    # Clean up
    node_num.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
  