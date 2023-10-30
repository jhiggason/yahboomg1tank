import os
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO

class LedControlNode(Node):
    def __init__(self, config):
        """
        Initializes the LedControlNode class.
        """
        super().__init__('led_control')

        # Load configuration from YAML file
        self.config = config

        # Subscribe to the /joy topic to receive joystick messages.
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.subscription_callback,
            10)

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Setup pins as output
        GPIO.setup(self.config['ServoPin'], GPIO.OUT)
        GPIO.setup(self.config['LED_R'], GPIO.OUT)
        GPIO.setup(self.config['LED_G'], GPIO.OUT)
        GPIO.setup(self.config['LED_B'], GPIO.OUT)

        # Initialize PWM on ServoPin with frequency of 50Hz
        self.pwm_servo = GPIO.PWM(self.config['ServoPin'], 50)
        self.pwm_servo.start(0)  # Start PWM with 0% duty cycle

        # Smoothing setup
        self.previous_servo_pos = 90  # Initialize previous servo position at center
        self.alpha = 0.2  # Smoothing factor, 0 < alpha < 1

        # Dead zone setup
        self.dead_zone = 0.05  # Dead zone around the center position

    # ... (other methods remain the same)

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Load configuration from YAML file
    config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # Create an instance of the LedControlNode class
    led_control_node = LedControlNode(config)

    try:
        # Spin the node to process incoming messages
        rclpy.spin(led_control_node)
    finally:
        # Clean up the node and shutdown the ROS 2 client library
        led_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
