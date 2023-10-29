import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class ServoControl(Node):
    """
    This class defines the ServoControl node which listens to the /cmd_vel topic
    and moves the servo motor based on the angular.z value of the Twist message.
    """
    def __init__(self):
        """Initialize the ServoControl node."""
        super().__init__('servo_control')

        # Define the GPIO pin for the servo motor.
        self.servo_pin = 23
        
        # Set the GPIO port to BCM encoding mode.
        GPIO.setmode(GPIO.BCM)

        # Ignore warning information.
        GPIO.setwarnings(False)

        # Initialize the servo motor pin.
        GPIO.setup(self.servo_pin, GPIO.OUT)
        
        # Initialize the PWM for the servo motor with 50Hz frequency.
        self.pwm_servo = GPIO.PWM(self.servo_pin, 50)
        self.pwm_servo.start(0)

        # Subscribe to the /cmd_vel topic with Twist message type.
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.subscription_callback,
            10,
        )

    def subscription_callback(self, msg):
        """
        Callback function for the /cmd_vel topic subscription.
        
        Parameters:
        msg (geometry_msgs.msg.Twist): The received Twist message.
        """
        # Extract the angular.z value from the Twist message.
        angular_z = msg.angular.z

        # Map the angular.z value to the servo motor range [0, 180].
        servo_pos = (angular_z + 1) * 90

        # Set the servo motor position.
        self.set_servo_position(servo_pos)

    def set_servo_position(self, position):
        """
        Set the position of the servo motor.
        
        Parameters:
        position (float): The desired position of the servo motor in degrees.
        """
        # Ensure the position is within the valid range [0, 180].
        position = max(0, min(position, 180))

        # Calculate the duty cycle for the given position.
        duty_cycle = 2.5 + 10 * position / 180

        # Change the PWM duty cycle to set the servo position.
        self.pwm_servo.ChangeDutyCycle(duty_cycle)

    def on_shutdown(self):
        """Cleanup the GPIO pins and stop the PWM on shutdown."""
        try:
            # Stop the PWM and clean up the GPIO pins.
            self.pwm_servo.stop()
            GPIO.cleanup()

        except Exception as e:
            self.get_logger().error(f'Error cleaning up GPIO pins: {e}')
            raise

def main(args=None):
    """Main function to initialize and run the ServoControl node."""
    try:
        # Initialize the ROS2 node.
        rclpy.init(args=args)
        node = ServoControl()

        try:
            # Spin the node until it is shut down.
            rclpy.spin(node)

        finally:
            # Stop the PWM and clean up the GPIO pins before shutting down.
            node.on_shutdown()
            node.destroy_node()
            rclpy.shutdown()

    except Exception as e:
        print(f'Error initializing ROS2 node: {e}')

if __name__ == '__main__':
    main()
