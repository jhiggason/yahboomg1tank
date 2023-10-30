# LED and Servo Control ROS2 Node

This ROS2 node controls an RGB LED and a servo motor based on joystick input. The servo motor's position is controlled by the right stick's horizontal axis, and the RGB LED's color can be changed with specific button presses.

## How it Works

When the node starts, it initializes the GPIO pins for the RGB LED and the servo motor. The servo motor is centered at 90 degrees, and the RGB LED is turned off.

The node then subscribes to the `/joy` topic, which publishes `sensor_msgs/msg/Joy` messages. These messages contain information about the state of the joystick, including the positions of the sticks and the states of the buttons.

The node has an `enabled` state, which is controlled by button 0. When button 0 is pressed, the node enters the `enabled` state, allowing the servo motor and RGB LED to be controlled by the joystick. When button 0 is released, the node exits the `enabled` state, the RGB LED is turned off, and the servo motor is centered.

In the `enabled` state, the position of the right stick's horizontal axis controls the position of the servo motor. The servo position is calculated based on the stick's position and smoothed with a low-pass filter to avoid sudden jumps.

Button 1, 2, and 3 are used to change the color of the RGB LED to red, white, and blue, respectively.

## Important Parts to Pay Attention To

1. **GPIO Pins:**
   - Make sure the GPIO pin numbers in the code match the physical connections on your Raspberry Pi.

2. **Servo Motor:**
   - The servo motor's position is controlled with PWM, and the pulse width is calculated based on the desired angle. Make sure your servo motor is compatible with the given pulse widths.

3. **RGB LED:**
   - The RGB LED is controlled by setting the GPIO pins to HIGH or LOW. Make sure your RGB LED is connected correctly, with the correct resistors if necessary.

4. **Smoothing:**
   - The servo motor's position is smoothed with a low-pass filter to avoid sudden jumps. You can adjust the `alpha` parameter to change the amount of smoothing.

5. **Dead Zone:**
   - A dead zone is used to prevent small movements of the joystick from affecting the servo motor's position. You can adjust the `dead_zone` parameter to change the size of the dead zone.

6. **ROS2 Topics and Messages:**
   - The node subscribes to the `/joy` topic and expects `sensor_msgs/msg/Joy` messages. Make sure your joystick is publishing to the correct topic and message type.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

