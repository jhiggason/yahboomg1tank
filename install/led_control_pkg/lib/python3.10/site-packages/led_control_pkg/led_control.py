import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO

class LedControlNode(Node):
    def __init__(self):
        """
        Initializes the LedControlNode class.
        """
        super().__init__('led_control')

        # Declare parameters
        self.declare_parameter('gpio_pins.servo', 23)
        self.declare_parameter('gpio_pins.led_r', 22)
        self.declare_parameter('gpio_pins.led_g', 27)
        self.declare_parameter('gpio_pins.led_b', 24)

        # Get parameters
        servo_pin = self.get_parameter('gpio_pins.servo').value
        led_r_pin = self.get_parameter('gpio_pins.led_r').value
        led_g_pin = self.get_parameter('gpio_pins.led_g').value
        led_b_pin = self.get_parameter('gpio_pins.led_b').value

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
        GPIO.setup(servo_pin, GPIO.OUT)
        GPIO.setup(led_r_pin, GPIO.OUT)
        GPIO.setup(led_g_pin, GPIO.OUT)
        GPIO.setup(led_b_pin, GPIO.OUT)

        # Initialize PWM on ServoPin with frequency of 50Hz
        self.pwm_servo = GPIO.PWM(servo_pin, 50)
        self.pwm_servo.start(0)  # Start PWM with 0% duty cycle

        # Smoothing setup
        self.previous_servo_pos = 90  # Initialize previous servo position at center
        self.alpha = 0.2  # Smoothing factor, 0 < alpha < 1

        # Dead zone setup
        self.dead_zone = 0.05  # Dead zone around the center position

    # ... (rest of your methods)

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the LedControlNode class
    led_control_node = LedControlNode()

    try:
        # Spin the node to process incoming messages
        rclpy.spin(led_control_node)
    finally:
        # Clean up the node and shutdown the ROS 2 client library
        led_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
