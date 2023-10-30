import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO

class LedControlNode(Node):
    def __init__(self):
        super().__init__('led_control')  # Initialize the parent class (Node)

        # Declare parameters with default values
        self.declare_parameter('gpio_pins.servo', 23)
        self.declare_parameter('gpio_pins.led_r', 22)
        self.declare_parameter('gpio_pins.led_g', 27)
        self.declare_parameter('gpio_pins.led_b', 24)

        # Get parameters from the parameter server or use default values
        self.ServoPin = self.get_parameter('gpio_pins.servo').get_parameter_value().integer_value
        self.LED_R = self.get_parameter('gpio_pins.led_r').get_parameter_value().integer_value
        self.LED_G = self.get_parameter('gpio_pins.led_g').get_parameter_value().integer_value
        self.LED_B = self.get_parameter('gpio_pins.led_b').get_parameter_value().integer_value

        # Subscribe to the /joy topic to receive joystick messages.
        self.subscription = self.create_subscription(
            Joy,  # Message type
            '/joy',  # Topic name
            self.subscription_callback,  # Callback function
            10)  # QoS profile (Quality of Service)

        self.subscription  # Prevent unused variable warning
        
        # GPIO setup
        GPIO.setmode(GPIO.BCM)  # Set the GPIO mode to BCM (Broadcom SOC channel)
        GPIO.setwarnings(False)  # Disable warnings

        # Setup pins as output
        GPIO.setup(self.ServoPin, GPIO.OUT)
        GPIO.setup(self.LED_R, GPIO.OUT)
        GPIO.setup(self.LED_G, GPIO.OUT)
        GPIO.setup(self.LED_B, GPIO.OUT)

        # Initialize PWM on ServoPin with frequency of 50Hz
        global pwm_servo
        pwm_servo = GPIO.PWM(self.ServoPin, 50)
        pwm_servo.start(0)  # Start PWM with 0% duty cycle

        # Smoothing setup
        self.previous_servo_pos = 90  # Initialize previous servo position at center
        self.alpha = 0.2  # Smoothing factor, 0 < alpha < 1
        
        # Dead zone setup
        self.dead_zone = 0.05  # Dead zone around the center position

    def destroy_node(self):
        """
        Cleanup function that is called when the node is destroyed.
        """
        super().destroy_node()  # Call the destroy_node method of the parent class
        pwm_servo.stop()  # Stop the PWM
        GPIO.cleanup()  # Cleanup the GPIO pins

    # ... (rest of the code remains the same)

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2 Python client library
    led_control_node = LedControlNode()  # Create an instance of the LedControlNode class

    try:
        rclpy.spin(led_control_node)  # Enter the ROS2 loop
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully
    finally:
        led_control_node.destroy_node()  # Clean up the node
        rclpy.shutdown()  # Shutdown ROS2 Python client library

if __name__ == '__main__':
    main()  # Run the main function when the script is executed
