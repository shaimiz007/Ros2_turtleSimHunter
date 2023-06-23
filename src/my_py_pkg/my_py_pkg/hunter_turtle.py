import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
from time import sleep



class Hunter(Node):
    def __init__(self):
        super().__init__('hunter')
       
        self.client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Teeport service not available, waiting...')
        
        request = TeleportAbsolute.Request()
        request.x = 3.0
        request.y = 5.0
        request.theta = 1.0

        try:
            future = self.client.call_async(request)
            future.result()
        except Exception as e:
            self.get_logger().error(e)
   
        #spawn counting
        
        self.timer_=self.create_timer(5,self.count_spawn)
        
    def count_spawn(self):
        service_names = self.get_service_names_and_types()
        self.spawn_count = 0
        for service_name in service_names:
            if str(service_name[0])[:7]=="/spawn_":
                self.spawn_count += 1
        self.get_logger().info("Spawn number = " + str(int(self.spawn_count/6)))      
        
def main(args=None):
    rclpy.init(args=args)
    hunter_node = Hunter()
    rclpy.spin(hunter_node)
    hunter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
