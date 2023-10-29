import rclpy
from rclpy.node import Node

class led_control(Node):

    def __init__(self):
        super().__init__('led_control')
        self.get_logger().info('Node has been started')

def main(args=None):
    rclpy.init(args=args)

    node = led_control()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
