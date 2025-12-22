from example_interfaces.srv import AddTwoInts
# from my_ros_study_msgs.srv import ArithmeticOperator, MySrv

import rclpy
from rclpy.node import Node


class MinimalService(Node):
    def __init__(self):
        super().__init__("minimal_service")
        self.srv = self.create_service(
            AddTwoInts, "calculate", self.calculate_two_ints_callback
        )

    def calculate_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        a = request.a
        b = request.b

        sub = a - b
        mul = a * b
        if b != 0:
            div = a / b
        else:
            div = 0.0  # Handle division by zero

        self.get_logger().info("Incoming request\na: %d b: %d" % (request.a, request.b))
        self.get_logger().info("sum: %d + %d = %d" % (a, b, response.sum))
        self.get_logger().info('sub: %d - %d = %d' % (a, b, sub))
        self.get_logger().info('mul: %d * %d = %d' % (a, b, mul))
        self.get_logger().info('div: %d / %d = %.2f' % (a, b, div)) # %.2f는 소수점 2자리까지

        return response

def main():
    rclpy.init()
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
