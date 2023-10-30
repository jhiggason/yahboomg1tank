import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO

class LedControlNode(Node):
    def __init__(self):
        """
        Initializes the LedControlNode class.

        The __init__ method is a special method in Python classes. 
        It is run as soon as an object of a class is instantiated.
        This method is useful for any initialization you want to do with your object.
        """
        super().__init__('led_control')  # Initialize the parent class (Node)

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

    def set_servo_position(self, position):
        """
        Set the servo motor position with smoothing.

        Parameters:
        position (float): The target position for the servo motor.
        """
        # Smooth the position using a simple low-pass filter
        smoothed_position = self.alpha * position + (1 - self.alpha) * self.previous_servo_pos

        # Update the previous servo position for the next iteration
        self.previous_servo_pos = smoothed_position

        # Set the servo motor position
        pwm_servo.ChangeDutyCycle(2.5 + 10 * smoothed_position / 180)

    def set_led_color(self, r, g, b):
        """
        Set the RGB LED color.

        Parameters:
        r, g, b (int): The RGB values to set the LED color.
        """
        GPIO.output(self.LED_R, r)
        GPIO.output(self.LED_G, g)
        GPIO.output(self.LED_B, b)

    def subscription_callback(self, msg):
        """
        Callback function for the /joy topic subscription.
        
        Parameters:
        msg (sensor_msgs.msg.Joy): The received Joy message.
        """
        # Extract the value of the horizontal axis of the right stick
        right_stick_horizontal = msg.axes[3]

        # Map the horizontal axis value to the servo motor range [0, 180]
        servo_pos = (right_stick_horizontal + 1) * 90

        # Implement dead zone
        if abs(right_stick_horizontal) < self.dead_zone:
            servo_pos = 90  # Center position

        # Set the servo motor position with smoothing
        self.set_servo_position(servo_pos)

        # LED color control
        if msg.buttons[1] == 1:
            self.set_led_color(GPIO.HIGH, GPIO.LOW, GPIO.LOW)  # Red
        elif msg.buttons[2] == 1:
            self.set_led_color(GPIO.HIGH, GPIO.HIGH, GPIO.HIGH)  # White
        elif msg.buttons[3] == 1:
            self.set_led_color(GPIO.LOW, GPIO.LOW, GPIO.HIGH)  # Blue

def main(args=None):
    """
    Main function to initialize the node and start the ROS2 loop.
    """
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