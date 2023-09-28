import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import lgpio as GPIO

class TankControl(Node):

    def __init__(self):
        super().__init__('tank_control')

        # Set up the GPIO pins for the left and right motors
        self.left_motor_pins = [20, 21, 16]  # Forward, Reverse, PWM
        self.right_motor_pins = [19, 26, 13]  # Forward, Reverse, PWM

        try:
            # Initialize the GPIO library
            GPIO.set_internal_socket_port(8888)
            GPIO.set_debug_level(0)
            GPIO.initialize()

            # Set up the left and right motor pins as outputs
            for pin in self.left_motor_pins + self.right_motor_pins:
                GPIO.set_mode(pin, GPIO.OUTPUT)

            # Set up the PWM for the left and right motors
            self.left_pwm = GPIO.pwm(self.left_motor_pins[2], 2000)  # 2000Hz frequency
            self.right_pwm = GPIO.pwm(self.right_motor_pins[2], 2000)  # 2000Hz frequency

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
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning

    def cmd_vel_callback(self, msg):
        # Parse the linear and angular velocities from the message
        linear = msg.linear.x
        angular = msg.angular.z

        # Calculate the left and right motor speeds based on the velocities
        left_speed = linear - angular
        right_speed = linear + angular

        # Set the left and right motor speeds using PWM
        self.left_pwm.ChangeDutyCycle(abs(left_speed) * 100)
        self.right_pwm.ChangeDutyCycle(abs(right_speed) * 100)

        # Set the direction of the left and right motors based on the sign of the speeds
        if left_speed >= 0:
            GPIO.write(self.left_motor_pins[0], GPIO.LOW)
            GPIO.write(self.left_motor_pins[1], GPIO.HIGH)
        else:
            GPIO.write(self.left_motor_pins[0], GPIO.HIGH)
            GPIO.write(self.left_motor_pins[1], GPIO.LOW)

        if right_speed >= 0:
            GPIO.write(self.right_motor_pins[0], GPIO.LOW)
            GPIO.write(self.right_motor_pins[1], GPIO.HIGH)
        else:
            GPIO.write(self.right_motor_pins[0], GPIO.HIGH)
            GPIO.write(self.right_motor_pins[1], GPIO.LOW)

def main(args=None):
    rclpy.init(args=args)

    tank_control = TankControl()

    rclpy.spin(tank_control)

    tank_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()