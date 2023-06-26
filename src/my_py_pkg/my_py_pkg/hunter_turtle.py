import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
from time import sleep
from turtlesim.msg import Pose


class Hunter(Node):
    def __init__(self):
        super().__init__("hunter")

        self.client = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Teeport service not available, waiting...")

        request = TeleportAbsolute.Request()
        request.x = 3.0
        request.y = 5.0
        request.theta = 1.0

        try:
            future = self.client.call_async(request)
            future.result()
        except Exception as e:
            self.get_logger().error(e)

        # Pose subscriber

        # spawn counting

        # self.timer_=self.create_timer(5,self.count_spawn)
        sleep(2.5)
        self.count_spawn()
        self.find_pose_topics()

    def count_spawn(self):
        service_names = self.get_service_names_and_types()
        self.spawn_count = 0
        for service_name in service_names:
            if str(service_name[0])[:7] == "/spawn_":
                self.spawn_count += 1
        self.get_logger().info("Spawn number = " + str(int(self.spawn_count / 6)))

    def find_pose_topics(self):
        topic_names = self.get_topic_names_and_types()
        pose_topics = [topic for topic, _ in topic_names if "pose" in topic]

        self.pose_subscriber = {}
        self.pose_list = set()
        for pose in pose_topics:
            self.pose_subscriber[pose] = self.create_subscription(
                Pose, pose, self.pose_callback, 10
            )

    def pose_callback(self, msg):
        x, y, theta = msg.x, msg.y, msg.theta
        pose = (x, y, theta)
        self.pose_list.add(pose)

    #    for i in self.pose_list:
    #       self.get_logger().info('x: %f y: %f theta: %f' % i)


def main(args=None):
    rclpy.init(args=args)
    hunter_node = Hunter()
    rclpy.spin(hunter_node)
    hunter_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
