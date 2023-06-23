#!/usr/bin/env python3
# Shebang line specifying the interpreter to be used for executing the script

import rclpy
# Importing the rclpy module for working with ROS 2 Python

from rclpy.node import Node
# Importing the Node class from rclpy.node module

from example_interfaces.srv import AddTwoInts
# Importing the AddTwoInts service message from the example_interfaces.srv module

def main(args=None):
    # Entry point of the script

    rclpy.init(args=args)
    # Initializing the ROS 2 Python client library

    node = Node("add_two_ints_no_opp")
    # Creating a Node object with the name "add_two_ints_no_opp"

    client = node.create_client(AddTwoInts, "add_two_ints")
    # Creating a client for the AddTwoInts service with the service name "add_two_ints"

    while not client.wait_for_service(1.0):
        # Waiting for the service to become available
        node.get_logger().Warning("Waiting for server Add Two Ints")

    request = AddTwoInts.Request()
    # Creating a request object of type AddTwoInts.Request

    request.a = 3
    request.b = 8
    # Setting the values of 'a' and 'b' in the request object

    future = client.call_async(request)
    # Calling the service asynchronously and getting a future object

    rclpy.spin_until_future_complete(node, future)
    # Spinning the node until the future is complete

    try:
        response = future.result()
        # Getting the response from the completed future

        node.get_logger().info(str(request.a) + " + " + str(request.b) + " = " + str(response.sum))
        # Logging the sum of 'a' and 'b' received in the response

    except Exception as e:
        node.get_logger().error("Service call failed %r" % (e,))
        # Logging an error message if the service call fails

    rclpy.shutdown()
    # Shutting down the ROS 2 Python client library

if __name__ == "__main__":
    main()
    # Calling the main function if the script is run directly
