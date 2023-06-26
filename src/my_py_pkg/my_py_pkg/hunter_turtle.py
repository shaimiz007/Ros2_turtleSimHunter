import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
from turtlesim.msg import Pose
from functools import partial
from math import sqrt


class Hunter(Node):
    def __init__(self):
        super().__init__("hunter")

        # Create client for the TeleportAbsolute service
        self.client = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Teleport service not available, waiting...")

        self.log_once = False
        self.timer_ = self.create_timer(0.5, self.find_pose_topics)

    def find_pose_topics(self):
        # Find topics that contain "pose" in their name
        topic_names = self.get_topic_names_and_types()
        pose_topics = [topic for topic, _ in topic_names if "pose" in topic]
        self.num_of_poses = len(pose_topics)

        # Create a dictionary to store pose subscribers and a set to store unique poses
        self.pose_subscriber = {}
        self.pose_list = set()

        for pose_topic in pose_topics:
            # Create a subscriber for each pose topic
            self.pose_subscriber[pose_topic] = self.create_subscription(
                Pose, pose_topic, partial(self.pose_callback, topic_name=pose_topic), 10 )

    def find_closest_distance_and_teleport(self, pos_set):
        min_distance = float("inf")
        closest_point = None
        for pose in pos_set:
            x_turtle, y_turtle, theta_turtle, topic_turtle = pose
            if topic_turtle == "/turtle1/pose":
                pos_set.remove(pose)
                break

        for pose in pos_set:
            x, y, theta, topic_name = pose
            distance = sqrt((x_turtle - x) ** 2 + (y_turtle - y) ** 2)  # distance
            if distance < min_distance:
                closest_point = (x, y, theta)
                self.call_teleport(x,y,theta,topic_name)
        return closest_point
    
    def call_teleport(self, x, y, theta,topic):
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta
        try:
            future = self.client.call_async(request)
            future.result()
            self.get_logger().info("Teleported to  "+topic[1:-5]+" (%f, %f, %f)" % (x, y, theta))
        except Exception as e:
            self.get_logger().error(e)


    def pose_callback(self, msg, topic_name):
        # Extract the pose values from the message
        x, y, theta = msg.x, msg.y, msg.theta
        pose = (x, y, theta, topic_name)
        # Add the pose to the set of poses
        self.pose_list.add(pose)
        if (
            len(self.pose_list) == self.num_of_poses
            and not self.log_once
            and self.num_of_poses >= 3
        ):
            self.find_closest_distance_and_teleport(self.pose_list)
            self.log_once = True


def main(args=None):
    rclpy.init(args=args)
    hunter_node = Hunter()
    rclpy.spin(hunter_node)
    hunter_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
