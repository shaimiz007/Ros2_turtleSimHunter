
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from my_robot_interfaces.srv import SetLed
 
class BatteryNode(Node): 
    def __init__(self):
        super().__init__("battery")
        self.get_logger().info("Battery started")
        # ANSI escape sequences for text colors
        RESET = '\033[0m'
        RED = '\033[31m'
        GREEN = '\033[32m'
        
        self.is_battery_full=True
        while True:
            self.get_logger().info(GREEN+"Battery Full"+RESET)
            self.call_set_led_server(status=False)
            time.sleep(4)
            
            self.get_logger().info(RED+"Battery Empty"+RESET)
            self.call_set_led_server(status=True)
            time.sleep(2)
            
     
    def call_set_led_server(self,status):
        self.client = self.create_client(SetLed, "LED_setting")   

        while not self.client.wait_for_service(1.0):             # Waiting for the service to become available
            self.get_logger().warning("Waiting for server LED_setting")

        request = SetLed.Request()         # Creating a request object of type AddTwoInts.Request
        request.powering_led_1 =status 
        future=self.client.call_async(request)         # Calling the service asynchronously without wating to response
        future.add_done_callback(self.callback_call_set_led)


    def callback_call_set_led(self,future):
        try:
            response = future.result()             # Getting the response from the completed future
            self.get_logger().info("Setting led sucsseed")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))             # Logging an error message if the service call fails 

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode() 
    rclpy.spin(node)
    rclpy.shutdown()
  
if __name__ == "__main__":
    main()

