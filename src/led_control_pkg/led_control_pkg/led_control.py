import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO

class ServoControlNode(Node):
    """
    A ROS2 node for controlling a servo motor using Joy messages from an Xbox controller.
    """
    
    def __init__(self):
        """
        Initialize the ServoControlNode.
        """
        super().__init__('servo_control_node')

        # Definition of servo pin
        self.servo_pin = 23
        
        # Set the GPIO port to BCM encoding mode.
        GPIO.setmode(GPIO.BCM)

        # Ignore warning information
        GPIO.setwarnings(False)

        # Servo pin is initialized into output mode
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.pwm_servo = GPIO.PWM(self.servo_pin, 50)
        self.pwm_servo.start(0)

        # Create a subscriber for /joy topic with callback function 'subscription_callback'.
        self.subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.subscription_callback,
            10,
        )
        
        # Log information to the console.
        self.get_logger().info('Servo control node has been started.')

    def subscription_callback(self, msg):
        """
        Callback function for the /joy topic subscription.
        
        Parameters:
        msg (sensor_msgs.msg.Joy): The received Joy message.
        """
        # Extract the value of the first axis (left stick horizontal) from the Joy message.
        axis_value = msg.axes[0]

        # Map the axis value to the servo motor range [0, 180].
        servo_pos = (axis_value + 1) * 90

        # Set the servo motor position.
        self.set_servo_position(servo_pos)

    def set_servo_position(self, pos):
        """
        Set the position of the servo motor.
        
        Parameters:
        pos (float): The desired position of the servo motor in degrees.
        """
        duty_cycle = 2.5 + 10 * pos / 180
        self.pwm_servo.ChangeDutyCycle(duty_cycle)


def main(args=None):
    rclpy.init(args=args)

    servo_control_node = ServoControlNode()

    rclpy.spin(servo_control_node)

    # Cleanup and shutdown.
    servo_control_node.pwm_servo.stop()
    GPIO.cleanup()
    servo_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
