import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from random import uniform, choice
from math import pi

import os

current_directory = os.getcwd()
print(current_directory)

# Read the names from the text file
def random_name_selctor():
    with open('/home/vboxuser/ros2_ws/install/my_robot_bringup/share/my_robot_bringup/launch/names1', 'r') as file:
    # with open("install/my_robot_bringup/share/my_robot_bringup/launch/names1", "r"    ) as file:
        # with open('names1.txt', 'r') as file:
        names = file.read().strip().split(",")
    # Randomly select a name
    random_name = choice(names).strip()
    # Print the randomly selected name
    return random_name

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
        self.spawn_request.name = random_name_selctor()

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
