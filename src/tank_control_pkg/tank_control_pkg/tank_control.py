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
        # Initialize the Node with the name 'tank_control'
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
        left_speed = self.linear_x - self.angular_z
        right_speed = self.linear_x + self.angular_z

        # Normalize speeds to be within -100 to 100 range
        left_speed = max(min(left_speed, 100), -100)
        right_speed = max(min(right_speed, 100), -100)

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


    def timer_callback(self):
        """
        Callback function executed periodically to stop the motors if no command is received.
        """
        # Check the elapsed time since the last message was received
        elapsed_time = time() - self.last_msg_time

        # If the elapsed time is greater than 0.1 seconds, stop the motors
        if elapsed_time >= 0.1:
            self.stop_motors()

    def drive(self, motor_pins, forward, reverse, speed):
        """
        Drive the tank in the specified direction and speed.
        
        Parameters:
        - motor_pins (list): Pins controlling the motor (e.g., left or right).
        - forward (bool): Drive forward.
        - reverse (bool): Drive backward.
        - speed (int): Speed to drive (0-100).
        """
        try:
            # Ensure speed is within 0-100 range
            speed = min(max(speed, 0), 100)

            # Set the GPIO pins for motor direction
            GPIO.output(motor_pins[0], forward)
            GPIO.output(motor_pins[1], reverse)

            # Set the PWM signal for motor speed
            if motor_pins == self.left_motor_pins:
                self.left_pwm.ChangeDutyCycle(speed)
            else:
                self.right_pwm.ChangeDutyCycle(speed)

        except Exception as e:
            # Log any errors during motor control
            self.get_logger().error('Error setting motor pins: %s' % str(e))
            raise

    def stop_motors(self):
        """
        Stop both tank motors and set GPIO pins to low.
        """
        try:
            # Stop the motors by setting the GPIO pins to low and the PWM signals to 0
            GPIO.output(self.left_motor_pins[0], False)
            GPIO.output(self.left_motor_pins[1], False)
            GPIO.output(self.right_motor_pins[0], False)
            GPIO.output(self.right_motor_pins[1], False)
            self.left_pwm.ChangeDutyCycle(0)
            self.right_pwm.ChangeDutyCycle(0)

        except Exception as e:
            # Log any errors during motor stop
            self.get_logger().error('Error stopping motors: %s' % str(e))
            raise


    def on_shutdown(self):
        """
        Actions to perform during node shutdown, including cleaning up GPIO.
        """
        try:
            # Stop the motors and clean up the GPIO pins
            self.stop_motors()
            GPIO.cleanup()

        except Exception as e:
            # Log any errors during GPIO cleanup
            self.get_logger().error('Error cleaning up GPIO pins: %s' % str(e))
            raise

# Define the main function
def main(args=None):
    """
    Main function to initialize and run the TankControl ROS2 node.
    
    Parameters:
    - args (list, optional): Command-line arguments passed to rclpy.init(). Default is None.
    """
    try:
        # Initialize the ROS2 node
        rclpy.init(args=args)
        node = TankControl()

        try:
            # Spin the node until it is shut down
            rclpy.spin(node)

        finally:
            # Stop the motors and clean up the GPIO pins before shutting down
            node.on_shutdown()
            node.destroy_node()
            rclpy.shutdown()

    except Exception as e:
        # Log any errors during ROS2 node initialization
        print('Error initializing ROS2 node: %s' % str(e))

# Check if the script is being run directly
if __name__ == '__main__':
    # Execute the main function
    main()
