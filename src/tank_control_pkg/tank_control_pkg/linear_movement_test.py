import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import sleep

class LinearMovementTest(Node):
    def __init__(self):
        super().__init__('linear_movement_test')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Run at 10 Hz
        self.start_time = self.get_clock().now()
        self.duration = 1.0  # Run for 1 seconds

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed_time = (now - self.start_time).nanoseconds / 1e9  # Convert nanoseconds to seconds

        msg = Twist()
        if elapsed_time < self.duration:
            msg.linear.x = 1.0  # Set desired linear velocity to 1 m/s
            self.get_logger().info('Publishing: Linear velocity: "%s"' % msg.linear.x)
        else:
            msg.linear.x = 0.0  # Stop the robot after 5 seconds
            self.get_logger().info('Publishing: Linear velocity: "%s"' % msg.linear.x)
            sleep(1)  # Give some time for the stop message to be processed
            self.destroy_node()  # Then shut down the node

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    linear_movement_test = LinearMovementTest()
    rclpy.spin(linear_movement_test)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
