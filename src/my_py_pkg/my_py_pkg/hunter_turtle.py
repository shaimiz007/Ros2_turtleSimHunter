#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute, Kill
from turtlesim.msg import Pose
from functools import partial
from math import sqrt
from time import sleep


class Hunter(Node):
    def __init__(self):
        super().__init__("hunter")

        # Create client for the TeleportAbsolute service
        self.client = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Teleport service not available, waiting...")

        self.dead_turtles = set()
        self.timer_ = self.create_timer(1, self.find_pose_topics)

    def find_pose_topics(self):
        self.log_once = False # flag to calculate the closet distance only one in each iteration
      
        # Find topics that contain "pose" in their name
        topic_names = self.get_topic_names_and_types()
        pose_topics = [topic for topic, _ in topic_names if "pose" in topic and topic[1:-5] not in self.dead_turtles]
        self.num_of_poses = len(pose_topics)
        self.get_logger().info(f"Number of pose topics: {self.num_of_poses}")

        # Create a dictionary to store pose subscribers and a set to store unique poses
        self.pose_subscriber = {}
        self.pose_list = set()

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
            len(self.pose_list) == self.num_of_poses # not to calculate the distanse before all pose location created
            and not self.log_once # calculate the closest distance only once in each subsription iteration
            and self.num_of_poses >= 2 # minimum number of turtles to calculate the distance
        ):
            self.log_once = True
          #  self.get_logger().info(f"log_once before send it to find closest : {self.log_once}")
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
            x, y, theta, turtle_name = pose
            distance = sqrt((x_turtle - x) ** 2 + (y_turtle - y) ** 2)  # distance
            if distance < min_distance:
                closest_point = (x, y, theta)
        self.call_teleport(closest_point[0], closest_point[1], closest_point[2], turtle_name)
       # sleep(0.5)
        self.send_kill_request(turtle_name)
            
    def call_teleport(self, x, y, theta, turtle_name):
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta
        try:
            future = self.client.call_async(request)
            future.result()
            self.get_logger().info(
                "Teleported to " + turtle_name + " (%f, %f, %f)" % (x, y, theta)
            )
        except Exception as e:
            self.get_logger().error(e)

    # Kill turtle method
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


def main(args=None):
    rclpy.init(args=args)
    hunter_node = Hunter()
    rclpy.spin(hunter_node)
    hunter_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
