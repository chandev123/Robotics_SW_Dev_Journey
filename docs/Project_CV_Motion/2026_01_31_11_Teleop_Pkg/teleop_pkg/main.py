import rclpy
from rclpy.executors import MultiThreadedExecutor
from teleop_pkg.teleop_pkg import Minipublisher

def main(args=None):
    rclpy.init(args=args)
    node = Minipublisher()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
