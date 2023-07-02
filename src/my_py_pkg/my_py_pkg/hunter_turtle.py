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

        self.dead_turtles = set()   #list of spawners to kill
        self.timer_ = self.create_timer(1, self.find_pose_topics)

    def find_pose_topics(self):
        self.log_once = False  # flag to calculate the closest distance only once in each iteration

        # Find topics that contain "pose" in their name
        topic_names = self.get_topic_names_and_types()
        pose_topics = [topic for topic, _ in topic_names if "pose" in topic and topic[1:-5] not in self.dead_turtles]
        self.num_of_poses = len(pose_topics)
        self.get_logger().info(f"Number of pose topics: {self.num_of_poses}")

        # Create a dictionary to store pose subscribers and a set to store unique poses
        self.pose_subscriber = {}
        self.pose_list = set()

        class Animal():
            def __init__(self,name,x,y,theta):
                self.name = name
                self.x = x
                self.y = y
                self.theta = theta
                self.distance = 0
                
        for pose_topic in pose_topics:
            # Create a subscriber for each pose topic
            self.pose_subscriber[pose_topic] = self.create_subscription(
                Pose, pose_topic, partial(self.pose_callback, topic_name=pose_topic), 10
            )

    def pose_callback(self, msg, topic_name):
        # Extract the turtle name from the topic name
        turtle_name = topic_name[1:-5]
        # Add the pose to the set of poses
        pose = (msg.x, msg.y, msg.theta, turtle_name)
        self.pose_list.add(pose)

        if (
            len(self.pose_list) == self.num_of_poses  # not to calculate the distance before all pose locations are created
            and not self.log_once  # calculate the closest distance only once in each subscription iteration
            and self.num_of_poses >= 2  # minimum number of turtles to calculate the distance
        ):
            self.log_once = True
            self.find_closest_distance_teleport_kill(self.pose_list)

    def find_closest_distance_teleport_kill(self, pos_set):
        min_distance = float("inf")
        closest_point = None
        for pose in pos_set:
            x_turtle, y_turtle, theta_turtle, topic_turtle = pose
            if topic_turtle == "turtle1":
                pos_set.remove(pose)
                break

        for pose in pos_set:
            x_spawner, y_spawner, theta_spawner, spawner_name  = pose
            distance = sqrt((x_turtle - x_spawner) ** 2 + (y_turtle - y_spawner) ** 2)  # calculate distance
            if distance < min_distance:
                min_distance = distance
                closest_point = (x_spawner, y_spawner, theta_spawner,spawner_name)
        self.call_teleport(x_turtle, y_turtle, closest_point[0], closest_point[1])
        self.send_kill_request(closest_point[3])

    def call_teleport(self, x_turtle, y_turtle, x_kill, y_kill):
        x_route, y_route = self.create_route(x_turtle, y_turtle, x_kill, y_kill)
        request = TeleportAbsolute.Request()
        for i in range(len(x_route)):
            request.x = x_route[i]
            request.y = y_route[i]
            try:
                future = self.client.call_async(request)
                future.result()
            except Exception as e:
                self.get_logger().error(e)("Failed to call teleport service")

    def send_kill_request(self, turtle_name):
        kill_client = self.create_client(Kill, "kill")
        self.dead_turtles.add(turtle_name)

        while not kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Kill service not available. Waiting...")

        request = Kill.Request()
        request.name = turtle_name
        future = kill_client.call_async(request)

        if future.result() is None:
            self.get_logger().info("Turtle killed successfully: %s" % turtle_name)
        else:
            self.get_logger().info("Failed to kill turtle: %s" % turtle_name)

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

def main(args=None):
    rclpy.init(args=args)
    hunter_node = Hunter()
    rclpy.spin(hunter_node)
    hunter_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
