import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
from functools import partial

class SetBoolClient(Node):
    def __init__(self):
        super().__init__("bool_client")
        self.call_bool_server(True)
        
    def call_bool_server(self, True_Or_False):
        self.client = self.create_client(SetBool, "reset_number")
        
        while not self.client.wait_for_service(1.0):
            self.get_logger().warning("Waiting for server SetBool")
        
        request = SetBool.Request()
        request.data = True_Or_False
        future = self.client.call_async(request)
        future.add_done_callback(partial(self.callback_set_bool, data=True_Or_False))

    def callback_set_bool(self, future, data):
        try:
            response = future.result()
            self.get_logger().info("Reset counter: ")
            self.get_logger().info(response.message)
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % e)
def main(args=None):
    rclpy.init(args=args)
    node = SetBoolClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
