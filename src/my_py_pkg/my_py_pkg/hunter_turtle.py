import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
from turtlesim.msg import Pose
from functools import partial
from time import sleep



class Hunter(Node):
    def __init__(self):
        super().__init__("hunter")

        # Create client for the TeleportAbsolute service
        self.client = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Teleport service not available, waiting...")

        # Set the request values for teleportation
        request = TeleportAbsolute.Request()
        request.x = 3.0
        request.y = 5.0
        request.theta = 1.0

        try:
            future = self.client.call_async(request)
            future.result()
        except Exception as e:
            self.get_logger().error(e)

        # Count the number of spawn services
        self.log_once=False
        # Find pose topics and create subscribers for each topic
        
        self.find_pose_topics()
  
    def find_pose_topics(self):
        # Find topics that contain "pose" in their name
        topic_names = self.get_topic_names_and_types()
        pose_topics = [topic for topic, _ in topic_names if "pose" in topic]
        self.num_of_poses = len(pose_topics)
        self.get_logger().info("Number of pose topics: %d" % self.num_of_poses)

        # Create a dictionary to store pose subscribers and a set to store unique poses
        self.pose_subscriber = {}
        self.pose_list = set()
        
        for pose_topic in pose_topics:
            # Create a subscriber for each pose topic
            self.pose_subscriber[pose_topic] =  self.create_subscription(
                Pose, pose_topic,partial(self.pose_callback,topic_name=pose_topic) , 10
            )
        

    def pose_callback(self, msg,topic_name):
        # Extract the pose values from the message
        x, y, theta = msg.x, msg.y, msg.theta
        pose = (x, y, theta, topic_name)
                # Add the pose to the set of poses
        self.pose_list.add(pose)
        if  len(self.pose_list) ==self.num_of_poses and self.log_once==False and self.num_of_poses>=2:
            self.get_logger().info(str(self.pose_list))
            self.log_once=True
    
def main(args=None):
    rclpy.init(args=args)
    hunter_node = Hunter()
    rclpy.spin(hunter_node)
    hunter_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()