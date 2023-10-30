import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO

class LedControlNode(Node):
    def __init__(self):
        super().__init__('led_control')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.subscription_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.ServoPin = 23
        GPIO.setup(self.ServoPin, GPIO.OUT)
        global pwm_servo
        pwm_servo = GPIO.PWM(self.ServoPin, 50)
        pwm_servo.start(0)

        # Smoothing setup
        self.previous_servo_pos = 90  # Initialize previous servo position at center
        self.alpha = 0.2  # Smoothing factor, 0 < alpha < 1

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

    def subscription_callback(self, msg):
        """
        Callback function for the /joy topic subscription.
        
        Parameters:
        msg (sensor_msgs.msg.Joy): The received Joy message.
        """
        # Extract the value of the horizontal axis of the left stick
        left_stick_horizontal = msg.axes[0]

        # Map the horizontal axis value to the servo motor range [0, 180]
        servo_pos = (left_stick_horizontal + 1) * 90

        # Set the servo motor position with smoothing
        self.set_servo_position(servo_pos)

def main(args=None):
    rclpy.init(args=args)

    led_control_node = LedControlNode()

    rclpy.spin(led_control_node)

    led_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
