# Import necessary libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
from time import time

# Define the TankControl class, inheriting from the Node class of rclpy
class TankControl(Node):
    """
    Class for controlling tank motion using ROS2 and Raspberry Pi GPIO.
    """
    
    def __init__(self):
        """
        Initialize the TankControl node, setup GPIO pins, and create ROS2 subscriptions/timers.
        """
        super().__init__('tank_control')

        # Define the GPIO pins for the left and right motors
        self.left_motor_pins = [20, 21, 16]  # Forward, Reverse, PWM
        self.right_motor_pins = [19, 26, 13]  # Forward, Reverse, PWM

        try:
            # Set the GPIO mode to BCM (Broadcom SOC channel)
            GPIO.setmode(GPIO.BCM)
            
            # Set up individual pins as output
            for pin in self.left_motor_pins + self.right_motor_pins:
                GPIO.setup(pin, GPIO.OUT)

            # Set up the PWM for the left and right motors
            self.left_pwm = GPIO.PWM(self.left_motor_pins[2], 2000)  # 2000Hz frequency
            self.right_pwm = GPIO.PWM(self.right_motor_pins[2], 2000)  # 2000Hz frequency

            # Start the PWM with 0% duty cycle (motor off)
            self.left_pwm.start(0)
            self.right_pwm.start(0)

        except Exception as e:
            # Log any errors during GPIO setup
            self.get_logger().error('Error setting up GPIO pins: %s' % str(e))
            raise

        # Subscribe to the /cmd_vel topic with the message type Twist and set the callback function
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.subscription_callback,
            10
        )

        # Create a timer to run the callback function at 10Hz to check for inactivity
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Initialize the time of the last message received
        self.last_msg_time = time()

    def subscription_callback(self, msg):
        """
        Callback to handle incoming ROS2 messages and control the tank motion.
        
        Parameters:
        - msg (Twist): The incoming ROS2 message containing the tank's desired motion parameters.
        """
        # Extract linear and angular velocities from the message
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

        # Calculate left and right wheel speeds
        angular_speed_amplified = self.angular_z * 1.5  # amplify the angular speed
        if self.linear_x >= 0:  # Forwards or stationary
            left_speed = self.linear_x - angular_speed_amplified
            right_speed = self.linear_x + angular_speed_amplified
        else:  # Backwards
            left_speed = self.linear_x + angular_speed_amplified
            right_speed = self.linear_x - angular_speed_amplified

        # Normalize speeds to be within -100 to 100 range
        left_speed = self.map_range(left_speed, -1.0, 1.0, -100, 100)  # assuming -1.0 and 1.0 are min and max possible speeds
        right_speed = self.map_range(right_speed, -1.0, 1.0, -100, 100)

        # Control the left motor
        if left_speed > 0:  # Drive forward
            self.drive(self.left_motor_pins, True, False, abs(left_speed))
        elif left_speed < 0:  # Drive backward
            self.drive(self.left_motor_pins, False, True, abs(left_speed))
        else:  # Stop
            self.stop_motors(self.left_motor_pins)

        # Control the right motor
        if right_speed > 0:  # Drive forward
            self.drive(self.right_motor_pins, True, False, abs(right_speed))
        elif right_speed < 0:  # Drive backward
            self.drive(self.right_motor_pins, False, True, abs(right_speed))
        else:  # Stop
            self.stop_motors(self.right_motor_pins)

        # Update the time of the last message received
        self.last_msg_time = time()

    def map_range(self, value, in_min, in_max, out_min, out_max):
        """
        Map a value from one range to another.

        Parameters:
        - value: The value to be mapped.
        - in_min: Minimum value of the input range.
        - in_max: Maximum value of the output range.
        - out_min: Minimum value of the output range.
        - out_max: Maximum value of the output range.
        
        Returns:
        - The mapped value.
        """
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def timer_callback(self):
        """
        Callback function executed periodically to stop the motors if no command is received.
        """
        # Check the elapsed time since the last message was received
        elapsed_time = time() - self.last_msg_time

        # If the elapsed time is greater than 0.1 seconds, stop the motors
        if elapsed_time > 0.1:
            self.stop_motors(self.left_motor_pins)
            self.stop_motors(self.right_motor_pins)

    def drive(self, pins, fwd, rev, speed):
        """
        Set motor direction and speed.

        Parameters:
        - pins: The GPIO pins associated with the motor.
        - fwd: Boolean indicating whether to drive forward.
        - rev: Boolean indicating whether to drive reverse.
        - speed: The speed to drive the motor (0 to 100).
        """
        GPIO.output(pins[0], fwd)
        GPIO.output(pins[1], rev)
        pins[2].ChangeDutyCycle(speed)

    def stop_motors(self, pins):
        """
        Stop the motors associated with the given GPIO pins.

        Parameters:
        - pins: The GPIO pins associated with the motor to be stopped.
        """
        GPIO.output(pins[0], False)
        GPIO.output(pins[1], False)
        pins[2].ChangeDutyCycle(0)

    def __del__(self):
        """
        Destructor to clean up GPIO pins when the object is deleted.
        """
        self.get_logger().info('Cleaning up GPIO pins.')
        GPIO.cleanup()


def main(args=None):
    """
    Main function to initialize the ROS2 node and run the TankControl class.
    """
    rclpy.init(args=args)
    tank_control = TankControl()
    rclpy.spin(tank_control)

    # Destroy the node explicitly (optional)
    tank_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
