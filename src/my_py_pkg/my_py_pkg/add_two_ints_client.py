#!/usr/bin/env python3
# Shebang line specifying the interpreter to be used for executing the script

import rclpy # Importing the rclpy module for working with ROS 2 Python
from rclpy.node import Node # Importing the Node class from rclpy.node module
from example_interfaces.srv import AddTwoInts # Importing the AddTwoInts service message from the example_interfaces.srv module
from functools import partial

class AddTwoIntClientNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")  
               # Initializing the parent Node class with the node name "add_two_ints_client"
        self.call_add_two_ints_server(6,7)
        self.call_add_two_ints_server(6,78)
        self.call_add_two_ints_server(-6,7)
        
    def call_add_two_ints_server(self,a,b):
        self.client = self.create_client(AddTwoInts, "add_two_ints")         # Creating a client for the AddTwoInts service with the service name "add_two_ints"

        while not self.client.wait_for_service(1.0):             # Waiting for the service to become available
            self.get_logger().warning("Waiting for server Add Two Ints")

        request = AddTwoInts.Request()         # Creating a request object of type AddTwoInts.Request

        request.a =a 
        request.b = b         # Setting the values of 'a' and 'b' in the request object

        future = self.client.call_async(request)         # Calling the service asynchronously and getting a future object
        future.add_done_callback(partial(self.callback_call_add_two_ints,a=a,b=b))

    def callback_call_add_two_ints(self,future,a,b):
        try:
            response = future.result()             # Getting the response from the completed future
            self.get_logger().info(
                str(a) + " + " + str(b) + " = " + str(response.sum)             )
            # Logging the sum of 'a' and 'b' received in the response
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))             # Logging an error message if the service call fails


def main(args=None):
    rclpy.init(args=args)     # Initializing the ROS 2 Python client library

    node = AddTwoIntClientNode()     # Creating an instance of the AddTwoIntClientNode class

    rclpy.spin(node)     # Spinning the node

    rclpy.shutdown()     # Shutting down the ROS 2 Python client library


if __name__ == "__main__":
    main()                             # Calling the main function if the script is run directly
