# Import necessary libraries
import yaml # Used with yaml config file @ /src/params_pkg/params/robot_params.yaml # pip install pyYAML 
import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Node class for creating ROS 2 nodes
from geometry_msgs.msg import Twist  # Message type for sending velocity commands
import RPi.GPIO as GPIO  # Library for Raspberry Pi GPIO control
from time import time  # Time module for tracking time-related events

# Define the TankControl class, inheriting from the Node class of rclpy
class TankControl(Node):
    """
    Class for controlling tank motion using ROS2 and Raspberry Pi GPIO.
    This class creates a ROS2 node to control the movement of a tank-like robot
    by listening to velocity commands and translating them into GPIO signals
    for motor control.
    """
    
    def subscription_callback(self, msg):
        """
        Callback to handle incoming ROS2 messages and control the tank motion.

        Parameters:
        - msg (Twist): The incoming ROS2 message containing the tank's desired motion parameters.
        """
        # Limit the linear speed to the maximum speed of the robot
        msg.linear.x = min(msg.linear.x, 1.27)  # Ensuring robot doesn't exceed max speed

        # Extract linear and angular velocities from the message
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

        # Apply a correction factor to linear velocity for real-world adjustments
        correction_factor = 0.79  # Adjust this value based on empirical testing
        corrected_linear_x = self.linear_x * correction_factor

        # Calculate left and right wheel speeds based on linear and angular velocities
        angular_speed_amplified = self.angular_z * 1.5  # Enhance the effect of angular velocity
        if corrected_linear_x >= 0:  # Condition for moving forwards or staying stationary
            left_speed = corrected_linear_x - angular_speed_amplified
            right_speed = corrected_linear_x + angular_speed_amplified
        else:  # Condition for moving backwards
            left_speed = corrected_linear_x + angular_speed_amplified
            right_speed = corrected_linear_x - angular_speed_amplified

        # Normalize speeds to be within -100 to 100 range
        left_speed = self.map_range(left_speed, -1.0, 1.0, -100, 100)  # Map to motor speed range
        right_speed = self.map_range(right_speed, -1.0, 1.0, -100, 100)

        # Control the left motor based on calculated speed
        if left_speed > 0:  # Forward condition
            self.drive(self.left_motor_pins, True, False, abs(left_speed))
            self.get_logger().info(f'Left motor driving forward at speed: {abs(left_speed)}')
        elif left_speed < 0:  # Reverse condition
            self.drive(self.left_motor_pins, False, True, abs(left_speed))
            self.get_logger().info(f'Left motor driving backward at speed: {abs(left_speed)}')
        else:  # Stop condition
            self.stop_motors(self.left_motor_pins)
            self.get_logger().info(f'Left motor stopped')

        # Similarly, control the right motor
        if right_speed > 0:
            self.drive(self.right_motor_pins, True, False, abs(right_speed))
            self.get_logger().info(f'Right motor driving forward at speed: {abs(right_speed)}')
        elif right_speed < 0:
            self.drive(self.right_motor_pins, False, True, abs(right_speed))
            self.get_logger().info(f'Right motor driving backward at speed: {abs(right_speed)}')
        else:
            self.stop_motors(self.right_motor_pins)
            self.get_logger().info(f'Right motor stopped')

        # Update the time of the last message received
        self.last_msg_time = time()

    def __init__(self):
        """
        Initialize the TankControl node, setup GPIO pins, and create ROS2 subscriptions/timers.
        This function sets up GPIO for motor control, starts PWM, and creates a ROS2 subscription
        to listen to velocity commands. It also initializes a timer to handle inactivity.
        """
        super().__init__('tank_control')  # Initialize the ROS2 node with the name 'tank_control'

        self.config = self.load_yaml_config("/src/params_pkg/params/robot_params.yaml")

        # Define the GPIO pins for the left and right motors
        # Using GPIO pin values from the YAML file
        self.left_motor_pins = [
            self.config['gpio_pins']['left_motor']['forward'],
            self.config['gpio_pins']['left_motor']['back'],
            self.config['gpio_pins']['left_motor']['pwm']
        ]
        self.right_motor_pins = [
            self.config['gpio_pins']['right_motor']['forward'],
            self.config['gpio_pins']['right_motor']['back'],
            self.config['gpio_pins']['right_motor']['pwm']
        ]

        # Set up GPIO
        try:
            GPIO.setmode(GPIO.BCM)
            for pin in self.left_motor_pins + self.right_motor_pins:
                GPIO.setup(pin, GPIO.OUT)

            self.left_pwm = GPIO.PWM(self.left_motor_pins[2], self.config['robot_parameters']['pwm']['frequency_motors'])
            self.right_pwm = GPIO.PWM(self.right_motor_pins[2], self.config['robot_parameters']['pwm']['frequency_motors'])
            self.left_pwm.start(0)
            self.right_pwm.start(0)
        except Exception as e:
            self.get_logger().error('Error setting up GPIO pins: %s' % str(e))
            raise

        # Subscribe to the /cmd_vel topic with the message type Twist
        # This subscription will receive velocity commands for the robot
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.subscription_callback,
            10  # Queue size of 10
        )

        # Create a timer to run the callback function at 10Hz
        # This is used to check for inactivity and stop the motors if needed
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Initialize the time of the last message received
        self.last_msg_time = time()

    def load_yaml_config(self, file_path):
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
        
    def map_range(self, value, in_min, in_max, out_min, out_max):
        """
        Map a value from one range to another.

        Parameters:
        - value: The value to be mapped.
        - in_min: Minimum value of the input range.
        - in_max: Maximum value of the input range.
        - out_min: Minimum value of the output range.
        - out_max: Maximum value of the output range.
        
        Returns:
        - The mapped value, scaled to the output range.
        """
        # Perform linear mapping from one range to another
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def timer_callback(self):
        """
        Callback function executed periodically to stop the motors if no command is received.
        This function checks if a set time has passed since the last command was received.
        If the time exceeds a threshold, it stops the motors to prevent unwanted movement.
        """
        # Calculate elapsed time since the last message was received
        elapsed_time = time() - self.last_msg_time
        self.get_logger().info(f'Elapsed time since last command: {elapsed_time} seconds')

        # Stop motors if the time since last command exceeds a threshold (0.1 seconds in this case)
        if elapsed_time >= 0.1:
            self.stop_motors(self.left_motor_pins)
            self.stop_motors(self.right_motor_pins)
            self.get_logger().info('Motors stopped due to inactivity')

    def drive(self, pins, fwd, rev, speed):
        """
        Set motor direction and speed.

        Parameters:
        - pins: The GPIO pins associated with the motor.
        - fwd: Boolean indicating whether to drive forward.
        - rev: Boolean indicating whether to drive reverse.
        - speed: The speed to drive the motor (0 to 100).
        
        This function controls the direction and speed of a motor using GPIO signals.
        """
        # Ensure speed is within 0-100 range
        speed = max(min(speed, 100), 0)
        
        # Set GPIO outputs for motor direction
        GPIO.output(pins[0], fwd)  # Forward control
        GPIO.output(pins[1], rev)  # Reverse control

        # Change duty cycle of PWM to control motor speed
        if pins == self.left_motor_pins:
            self.left_pwm.ChangeDutyCycle(speed)
        elif pins == self.right_motor_pins:
            self.right_pwm.ChangeDutyCycle(speed)

        # Log the motor direction and speed
        self.get_logger().info(f'Motor direction: {"forward" if fwd else "backward"}')
        self.get_logger().info(f'Motor speed set to: {speed}')

    def stop_motors(self, pins):
        """
        Stop the motors associated with the given GPIO pins.

        Parameters:
        - pins: The GPIO pins associated with the motor to be stopped.
        
        This function stops a motor by disabling the GPIO outputs and setting the PWM duty cycle to 0.
        """
        # Disable both forward and reverse controls
        GPIO.output(pins[0], False)
        GPIO.output(pins[1], False)

        # Set duty cycle to 0 to stop the motor
        if pins == self.left_motor_pins:
            self.left_pwm.ChangeDutyCycle(0)
        elif pins == self.right_motor_pins:
            self.right_pwm.ChangeDutyCycle(0)

    def __del__(self):
        """
        Destructor to clean up GPIO pins when the object is deleted.
        This method is called when the instance is being destroyed to ensure proper cleanup.
        """
        self.get_logger().info('Cleaning up GPIO pins.')
        GPIO.cleanup()  # Release all GPIO resources

def main(args=None):
    """
    Main function to initialize the ROS2 node and run the TankControl class.
    This function sets up ROS2, creates an instance of the TankControl class, and keeps it running.
    """
    rclpy.init(args=args)  # Initialize ROS 2
    tank_control = TankControl()  # Create an instance of the TankControl class
    rclpy.spin(tank_control)  # Keep the node active

    # Destroy the node explicitly (optional but recommended for clean exit)
    tank_control.destroy_node()
    rclpy.shutdown()  # Shutdown ROS 2

if __name__ == '__main__':
    main()  # Run the main function when the script is executed
