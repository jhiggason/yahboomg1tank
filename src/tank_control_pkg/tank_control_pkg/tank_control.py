import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO

class TankControl(Node):

    def __init__(self):
        super().__init__('tank_control')

        # Set up the GPIO pins for the left and right motors
        self.left_motor_pins = [20, 21, 16]  # Forward, Reverse, PWM
        self.right_motor_pins = [19, 26, 13]  # Forward, Reverse, PWM

        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.left_motor_pins, GPIO.OUT)
            GPIO.setup(self.right_motor_pins, GPIO.OUT)

            # Set up the PWM for the left and right motors
            self.left_pwm = GPIO.PWM(self.left_motor_pins[2], 2000)  # 2000Hz frequency
            self.right_pwm = GPIO.PWM(self.right_motor_pins[2], 2000)  # 2000Hz frequency

            # Start the PWM with 0% duty cycle
            self.left_pwm.start(0)
            self.right_pwm.start(0)

        except Exception as e:
            self.get_logger().error('Error setting up GPIO pins: %s' % str(e))
            raise

        # Subscribe to the /turtle1/cmd_vel topic
        self.subscription = self.create_subscription(
            String,
            '/turtle1/cmd_vel',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # Access the linear and angular velocity from the message data
        linear = msg.linear.x
        angular = msg.angular.z

        # Control the motors based on the linear and angular velocity
        if linear > 0:  # move forward
            self.drive(self.left_motor_pins, True, False, 100)  # 100% duty cycle for full speed
            self.drive(self.right_motor_pins, True, False, 100)
        elif linear < 0:  # move backward
            self.drive(self.left_motor_pins, False, True, 100)
            self.drive(self.right_motor_pins, False, True, 100)
        elif angular > 0:  # turn/spin left
            self.drive(self.left_motor_pins, False, True, 100)
            self.drive(self.right_motor_pins, True, False, 100)
        elif angular < 0:  # turn/spin right
            self.drive(self.left_motor_pins, True, False, 100)
            self.drive(self.right_motor_pins, False, True, 100)
        else:
            self.stop_motors()

    def drive(self, motor_pins, forward, reverse, speed):
        # Set the motor pins and PWM duty cycle
        try:
            GPIO.output(motor_pins[0], forward)
            GPIO.output(motor_pins[1], reverse)
            pwm = motor_pins[2]
            pwm.ChangeDutyCycle(speed)

        except Exception as e:
            self.get_logger().error('Error setting motor pins: %s' % str(e))
            raise

    def stop_motors(self):
        # Stop the motors and clean up the GPIO pins
        try:
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
        # Stop the motors and clean up the GPIO pins when the node is shut down
        try:
            self.stop_motors()
            GPIO.cleanup()

        except Exception as e:
            self.get_logger().error('Error cleaning up GPIO pins: %s' % str(e))
            raise

def main(args=None):
    # Initialize the ROS2 node
    try:
        rclpy.init(args=args)

        # Create an instance of the TankControl class
        node = TankControl()

        # Spin the node until it is shut down
        try:
            rclpy.spin(node)

        finally:
            # Stop the motors and clean up the GPIO pins when the node is shut down
            node.on_shutdown()

            # Destroy the node and shut down ROS2
            node.destroy_node()
            rclpy.shutdown()

    except Exception as e:
        print('Error initializing ROS2 node: %s' % str(e))

if __name__ == '__main__':
    main()