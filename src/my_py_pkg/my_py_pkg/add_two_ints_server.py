#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

# Define a class AddTwoIntsServerNode that inherits from Node class
class AddTwoIntsServerNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_server")  # Initialize the Node with the name "add_two_ints_server"
        # Create a service named "add_two_ints" with the service type AddTwoInts
        # and associate it with the callback function "callback_add_two_ints"
        self.server_ = self.create_service(AddTwoInts, "add_two_ints", self.callback_add_two_ints)
        self.get_logger().info("Add two ints server has been started")
   
    def callback_add_two_ints(self, request, response):
        # Perform the addition operation by adding the values of request.a and request.b
        response.sum = request.a + request.b
        # Log the input values and the result
        self.get_logger().info(str(request.a) + " + " + str(request.b) + " = " + str(response.sum))
        # Return the response object with the calculated sum
        return response


def main(args=None):
    rclpy.init(args=args)
    # Create an instance of the AddTwoIntsServerNode class
    node = AddTwoIntsServerNode()
    # Start spinning the node, allowing it to process incoming requests
    rclpy.spin(node)
    # Clean up and shut down the node
    rclpy.shutdown()


if __name__ == "__main__":
    main()
