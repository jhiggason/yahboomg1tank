import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class TankControl(Node):

    def __init__(self):
        super().__init__('tank_control')

        # Set up the GPIO pins for the left and right motors
        self.left_motor_pins = [20, 21, 16]  # Forward, Reverse, PWM
        self.right_motor_pins = [19, 26, 13]  # Forward, Reverse, PWM

        try:
            GPIO.setmode(GPIO.BCM)
            
            # Set up individual pins
            for pin in self.left_motor_pins + self.right_motor_pins:
                GPIO.setup(pin, GPIO.OUT)

            # Set up the PWM for the left and right motors
            self.left_pwm = GPIO.PWM(self.left_motor_pins[2], 2000)  # 2000Hz frequency
            self.right_pwm = GPIO.PWM(self.right_motor_pins[2], 2000)  # 2000Hz frequency

            # Start the PWM with 0% duty cycle
            self.left_pwm.start(0)
            self.right_pwm.start(0)

        except Exception as e:
            self.get_logger().error('Error setting up GPIO pins: %s' % str(e))
            raise

        # Subscribe to the /turtle1/cmd_vel topic with correct message type
        self.subscription = self.create_subscription(
            Twist,
            '/turtle1/cmd_vel',
            self.listener_callback,
            10
        )

        # Create a timer to run the callback function at 2Hz
        self.timer = self.create_timer(0.5, self.timer_callback)

        # Initialize the flag to indicate if the message is clear
        self.msg_clear = False

    def listener_callback(self, msg):
        # Extract linear and angular velocities from the message
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        # Assuming linear_x drives forward/backward and linear_y performs strafe (left/right)
        # For tank bot, strafe might not be possible, but let's take it into account anyway
        if linear_x > 0:  # Drive forward
            self.drive(self.left_motor_pins, True, False, abs(linear_x))
            self.drive(self.right_motor_pins, True, False, abs(linear_x))
            self.msg_clear = False
        elif linear_x < 0:  # Drive backward
            self.drive(self.left_motor_pins, False, True, abs(linear_x))
            self.drive(self.right_motor_pins, False, True, abs(linear_x))
            self.msg_clear = False
        elif angular_z > 0:  # Turn left
            self.drive(self.left_motor_pins, False, True, abs(angular_z))
            self.drive(self.right_motor_pins, True, False, abs(angular_z))
            self.msg_clear = False
        elif angular_z < 0:  # Turn right
            self.drive(self.left_motor_pins, True, False, abs(angular_z))
            self.drive(self.right_motor_pins, False, True, abs(angular_z))
            self.msg_clear = False
        else:
            self.msg_clear = True

    def timer_callback(self):
        # If no new message has been received, stop the motors
        if self.msg_clear:
            self.stop_motors()

    def drive(self, motor_pins, forward, reverse, speed):
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
            self.get_logger().error('Error setting motor pins: %s' % str(e))
            raise

    def stop_motors(self):
        try:
            # Stop the motors by setting the GPIO pins to low and the PWM signals to 0
            GPIO.output(self.left_motor_pins[0], False)
            GPIO.output(self.left_motor_pins[1], False)
            GPIO.output(self.right_motor_pins[0], False)
            GPIO.output(self.right_motor_pins[1], False)
            self.left_pwm.ChangeDutyCycle(0)
            self.right_pwm.ChangeDutyCycle(0)

        except Exception as e:
            self.get_logger().error('Error stopping motors: %s' % str(e))
            raise

    def on_shutdown(self):
        try:
            # Stop the motors and clean up the GPIO pins
            self.stop_motors()
            GPIO.cleanup()

        except Exception as e:
            self.get_logger().error('Error cleaning up GPIO pins: %s' % str(e))
            raise

def main(args=None):
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
        print('Error initializing ROS2 node: %s' % str(e))

if __name__ == '__main__':
    main()