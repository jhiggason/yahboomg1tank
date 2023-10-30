from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # tank_control node
        Node(
            package='tank_control_pkg',
            executable='tank_control',
            name='tank_control',
            output='screen'
        ),

        # led_control node
        Node(
            package='led_control_pkg',
            executable='led_control',
            name='led_control',
            parameters=[{'gpio_pins': '/config/gpio_params.yaml'}],  # Assuming you have the gpio_params.yaml in a config folder within led_control_pkg
            output='screen'
        ),
    ])
