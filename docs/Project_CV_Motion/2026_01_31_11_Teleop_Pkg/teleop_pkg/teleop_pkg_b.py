# DR_init과 ROS2 통신

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

class Minipublisher(Node):
    def __init__(self):
        super().__init__("mini_publisher")
        self.publisher1 = self.create_publisher(String, "test_topic1", 10)
        self.publisher2 = self.create_publisher(String, "test_topic2", 10)
        self.client = self.create_service(SetBool, "control_test", self.service_callback)
        self.timer1 = self.create_timer(1, self.timer_callback1)
        self.timer2 = self.create_timer(1, self.timer_callback2)

    def timer_callback1(self):
        msg = String()
        msg.data = "Hello world"
        self.publisher1.publish(msg)
        self.get_logger().info("Publishing: %s" % msg.data)

    def timer_callback2(self):
        msg = String()
        msg.data = "Rokey"
        self.publisher2.publish(msg)
        self.get_logger().info("Publishing: %s" % msg.data)

    def service_callback(self, request, response):
        response.success = True
        self.get_logger().info('Service: %s' % response.success)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Minipublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

