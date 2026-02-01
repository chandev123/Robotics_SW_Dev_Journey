# DR_init과 ROS2 통신

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class MiniTeleopClient(Node):
    def __init__(self):
        super().__init__("mini_teleop_client")
        self.client = self.create_client(SetBool, "control_test")

        while not self.client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning('The control_test service not available.')

    def send_request(self):
        service_request=SetBool.Request()
        service_request.data = True
        futures = self.client.call_async(service_request)
        return futures

def main(args=None):
    rclpy.init(args=args)
    node = MiniTeleopClient()
    future = node.send_request()
    user_trigger = True
    try:
        while rclpy.ok():
            if user_trigger is True:
                rclpy.spin_once(node)
                if future.done():
                    try:
                        service_response = future.result()
                    except Exception as e:
                        node.get_logger().warn('Service call failed: {}'.format(str(e)))
                    else:
                        node.get_logger().info(
                            'Result: {}'.format(service_response.success))
                        user_trigger = False
            else:
                input('Press Enter for next service call.')
                future = node.send_request()
                user_trigger = True

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
