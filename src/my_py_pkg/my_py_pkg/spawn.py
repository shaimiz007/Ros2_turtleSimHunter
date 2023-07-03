import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from random import uniform
from math import pi
import os

current_directory = os.getcwd()
print(current_directory)

class MyTurtleSpawner(Node):
    def __init__(self):
        super().__init__('spawner')
        self.spawn_client = self.create_client(Spawn, 'spawn')

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting...')

        self.spawn_request = Spawn.Request()
         
        # Get parameter values
        self.spawn_request.x = uniform(0.5, 9.5)
        self.spawn_request.y = uniform(0.5, 9.5)
        self.spawn_request.theta = uniform(0, 2 *pi)
        self.declare_parameter('name','temp')
        self.spawn_request.name = self.get_parameter('name').value
        future = self.spawn_client.call_async(self.spawn_request)
        future.result()

def main(args=None):
    rclpy.init(args=args)
    spawner_node = MyTurtleSpawner()
    rclpy.spin(spawner_node)
    spawner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
