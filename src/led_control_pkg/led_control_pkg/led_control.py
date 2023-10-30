import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO

class LedControlNode(Node):
    """
    A ROS2 node for controlling an LED and servo motor based on joystick input.
    """
    def __init__(self):
        """
        Initialize the LedControlNode class.
        """
        # Initialize the parent class (Node)
        super().__init__('led_control')

        # Subscribe to the /joy topic to receive joystick messages
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.subscription_callback,
            10)

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Define pin numbers for the servo motor and LED
        self.ServoPin = 23
        self.LED_R = 22
        self.LED_G = 27
        self.LED_B = 24

        # Setup pins as output
        GPIO.setup(self.ServoPin, GPIO.OUT)
        GPIO.setup(self.LED_R, GPIO.OUT)
        GPIO.setup(self.LED_G, GPIO.OUT)
        GPIO.setup(self.LED_B, GPIO.OUT)

        # Initialize PWM on ServoPin with frequency of 50Hz
        self.pwm_servo = GPIO.PWM(self.ServoPin, 50)
        self.pwm_servo.start(0)

        # Enable/disable control
        self.enabled = False
        
        # Smoothing setup
        self.alpha = 0.2
        self.previous_servo_pos = 90

        # Center the servo on startup
        self.set_servo_position(90)

        # Dead zone setup
        self.dead_zone = 0.05


    def destroy_node(self):
        """
        Cleanup function that is called when the node is destroyed.
        """
        super().destroy_node()
        self.pwm_servo.stop()
        GPIO.cleanup()

    def set_servo_position(self, position):
        """
        Set the servo motor position with smoothing.
        """
        if not self.enabled:
            return  # Disable the servo when not enabled
        smoothed_position = self.alpha * position + (1 - self.alpha) * self.previous_servo_pos
        self.previous_servo_pos = smoothed_position
        pulsewidth = (smoothed_position * 11) + 500
        duty_cycle = (pulsewidth / 20000) * 100
        self.pwm_servo.ChangeDutyCycle(duty_cycle)

    def set_led_color(self, r, g, b):
        """
        Set the RGB LED color.
        """
        GPIO.output(self.LED_R, r)
        GPIO.output(self.LED_G, g)
        GPIO.output(self.LED_B, b)

    def subscription_callback(self, msg):
        """
        Callback function for the /joy topic subscription.
        """
        self.enabled = msg.buttons[0] == 1 # Monitor button at index [0] - typically the "A" button on xbox controllers
        
        if self.enabled:
            right_stick_horizontal = msg.axes[3]
            servo_pos = (right_stick_horizontal + 1) * 90

            if abs(right_stick_horizontal) < self.dead_zone:
                servo_pos = 90

            self.set_servo_position(servo_pos)

            if msg.buttons[1] == 1:
                self.set_led_color(GPIO.HIGH, GPIO.LOW, GPIO.LOW)  # Turn LED to red
            else:
                self.set_led_color(GPIO.LOW, GPIO.HIGH, GPIO.LOW)  # You can change this to any color you want when the enable button isn't pressed
        else:
            self.set_led_color(GPIO.LOW, GPIO.LOW, GPIO.LOW)  # Turn off the LED when disabled
            self.set_servo_position(90)  # Center the servo when disabled

def main(args=None):
    """
    Main function to initialize the node and start the ROS2 loop.
    """
    rclpy.init(args=args)  # Initialize ROS2
    led_control_node = LedControlNode()  # Create an instance of the LedControlNode class

    try:
        rclpy.spin(led_control_node)  # Enter the ROS2 loop
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully
    finally:
        led_control_node.destroy_node()  # Clean up the node
        rclpy.shutdown()  # Shutdown ROS2

if __name__ == '__main__':
    main()  # Run the main function when the script is executed
