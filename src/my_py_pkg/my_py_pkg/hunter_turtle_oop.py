#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute, Kill
from turtlesim.msg import Pose
from functools import partial
from math import sqrt
from random import uniform
import numpy as np

class Hunter(Node):
    def __init__(self):
        super().__init__("hunter")
      
        # Create client for the TeleportAbsolute service
        self.client = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Teleport service not available, waiting...")

        self.dead_animals = set()   #list of spawners  that been killed
        self.timer_ = self.create_timer(1, self.find_pose_topics)

    def find_pose_topics(self):
        self.log_once = False  # flag to calculate the closest distance only once in each iteration

        # Find topics that contain "pose" in their name
        topic_names = self.get_topic_names_and_types()
        pose_topics = [topic for topic, _ in topic_names if "pose" in topic and topic[1:-5] not in self.dead_animals]
        self.num_of_poses = len(pose_topics)
        self.get_logger().info(f"Number of pose spawners: {self.num_of_poses-1}")

        # Create a dictionary to store pose subscribers and a set to store unique poses
        self.pose_subscriber = {}
        self.animals_lists = set()

        for pose_topic in pose_topics:
            # Create a subscriber for each pose topic
            self.pose_subscriber[pose_topic] = self.create_subscription(
                Pose, pose_topic, partial(self.pose_callback, topic_name=pose_topic), 10
            )

    def pose_callback(self, msg, topic_name):
        animal=self.Animal(topic_name[1:-5],msg.x,msg.y,msg.theta)
        # Extract the turtle name from the topic name
        self.animals_lists.add(animal)

        if (
            len(self.animals_lists) == self.num_of_poses  # not to calculate the distance before all pose locations are created
            and not self.log_once  # calculate the closest distance only once in each subscription iteration
            and self.num_of_poses >= 2  # minimum number of turtles to calculate the distance
        ):
            self.log_once = True
            self.find_closest_distance_teleport_kill(self.animals_lists)

    def find_closest_distance_teleport_kill(self, animals_list):
        min_distance = float("inf")
        closest_point = None
        for animal in animals_list:
            if animal.name== "turtle1":
                self.turtle=animal
                animals_list.remove(animal)
                break

        for animal in animals_list:
            animal_dis=animal.distance(self.turtle)
            if  animal_dis< min_distance:
                min_distance = animal_dis
                closest_animal = animal
        self.call_teleport(closest_animal)
        self.send_kill_request(closest_animal)

    def call_teleport(self,animal):
        x_route, y_route = self.create_route(self.turtle.x, self.turtle.y, animal.x, animal.y)
        request = TeleportAbsolute.Request()
        for i in range(len(x_route)):
            request.x = x_route[i]
            request.y = y_route[i]
            try:
                future = self.client.call_async(request)
                future.result()
            except Exception as e:
                self.get_logger().error(e)("Failed to call teleport service")

    def send_kill_request(self, animal):
        kill_client = self.create_client(Kill, "kill")
        self.dead_animals.add(animal.name)

        while not kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Kill service not available. Waiting...")

        request = Kill.Request()
        request.name = animal.name
        
        future = kill_client.call_async(request)

        if future.result() is None:
            self.get_logger().info("Turtle killed successfully: %s" % animal.name)
        else:
            self.get_logger().info("Failed to kill turtle: %s" % animal.name)

    def create_route(self, X1, Y1, X2, Y2):
        NUM_FIT_POINTS = 4
        NUM_CURVE_POINTS = 2000
        N_POLYNOM = 3

        x_points = [uniform(X1, X2) for _ in range(NUM_FIT_POINTS)]
        if X1 < X2:
            x = [X1] + sorted(x_points) + [X2]
        else:
            x = [X1] + sorted(x_points, reverse=True) + [X2]

        y_points = [uniform(Y1, Y2) for _ in range(NUM_FIT_POINTS)]
        if Y1 < Y2:
            y = [Y1] + sorted(y_points) + [Y2]
        else:
            y = [Y1] + sorted(y_points, reverse=True) + [Y2]

        # Fit a polynomial curve of degree N to the given points
        coefficients = np.polyfit(x, y, N_POLYNOM)
        polynomial = np.poly1d(coefficients)

        # Generate x values to plot the curve
        x_plot = np.linspace(x[0], x[-1], NUM_CURVE_POINTS)

        # Calculate the corresponding y values using the polynomial curve
        y_plot = polynomial(x_plot)

        # Handling points outside the screen
        for n, y_val in enumerate(y_plot):
            if y_val < 0:
                y_plot[n] = -y_val
            elif y_val > 11:
                y_plot[n] = 22 - y_val
            else:
                y_plot[n] = y_val

        y_plot[0] = y[0]  # assign Y1 & Y2 to the first & last points of the curve
        y_plot[-1] = y[-1]
        return x_plot, y_plot
 
    class Animal:
        def __init__(self,name,x,y,theta):
            self.name = name
            self.x = x
            self.y = y
            self.theta = theta
        
        def distance (self,turtle):
            return sqrt((self.x-turtle.x)**2 + (self.y-turtle.y)**2)
        
        def __eq__(self, other):
                return self.name == other.name
         
        def __hash__(self):
            return hash(self.name)


def main(args=None):
    rclpy.init(args=args)
    hunter_node = Hunter()
    rclpy.spin(hunter_node)
    hunter_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
