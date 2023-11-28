# YAHBOOM G1 Tank ROS2

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]

<div align="center">
    <a href="https://github.com/jhiggason/YahBoomG1Tank">
        <img src="img/tank.png" alt="Logo" width="670" height="268">
    </a>
</div>

<div align="left">
    
## Overview
The YAHBOOM G1 Tank ROS2 project is a Python-based node designed for the YAHBOOM G1 Tank, built on the Raspberry Pi 4. This project provides a flexible and customizable platform for controlling the YAHBOOM G1 Tank, with support for the Yahboom 4WD expansion board for robot car, SKU: 6000300085, and related vehicles.

The project is built on top of the Robot Operating System 2 (ROS2), which provides a powerful and flexible framework for developing robotic applications. The YAHBOOM G1 Tank ROS2 project uses either the turtlesim teleop executable for basic control or a gamepad, such as an Xbox controller, for control.

For more information on how to get started with this project, please refer to the documentation. If you encounter any issues or have any questions, feel free to open a new issue on the project's GitHub page.


**Key Features:**
- Controlled via `turtlesim` teleop executable.
- Controlled via 'ros-humble-teleop-twist-joy` that uses the ros2 package joy to provide gamepad support- like an xbox controller. 
- Support for the "Yahboom 4WD expansion board for robot car, SKU: 6000300085" and related vehicles.
  
<a href="https://github.com/jhiggason/YahBoomG1Tank#getting-started"><strong>Explore the Documentation »</strong></a>  
<a href="https://github.com/jhiggason/YahBoomG1Tank">View Demo</a> |
<a href="https://github.com/jhiggason/YahBoomG1Tank/issues">Report Bug</a> |
<a href="https://github.com/jhiggason/YahBoomG1Tank/issues">Request Feature</a>

</div>

## Table of Contents

- [About The Project](#about-the-project)
  - [Built With](#built-with)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation and Setup](#installation-and-setup)
  - [First Use](#first-use)
- [How It Works](#how-it-works)
- [Roadmap](#roadmap)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## About The Project

![Project Image](https://hackster.imgix.net/uploads/attachments/1418126/_y4NHHNkT2t.blob?auto=compress%2Cformat&w=900&h=675&fit=min)

### Built With
- [Python3][Python-url]
- [ROS2 Humble][ROS-url]

<!-- GETTING STARTED -->
## Getting Started

This guide will help you set up and run the ROS2 Python node for the YAHBOOM G1 Tank with Raspberry Pi 4.

- [LED Control Package](https://github.com/jhiggason/yahboomg1tank/tree/main/src/led_control_pkg)
- [Tank Control Package](https://github.com/jhiggason/yahboomg1tank/tree/main/src/tank_control_pkg)
  
### Prerequisites

#### Hardware:
- **YAHBOOM G1 Tank**: Ensure it's fully assembled and tested for basic operations.
- **Raspberry Pi 4**: Properly interfaced with the G1 Tank.

#### Software:
- **Ubuntu 22 for Raspberry Pi**: Installed and set up on your Raspberry Pi 4. [Ubuntu installation guide](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#1-overview).
- **ROS2 Humble-desktop**: Installed and sourced following the official [ROS2 installation guide](https://index.ros.org/doc/ros2/Installation/Humble/).
- **Colcon**: Required for building and testing ROS packages. Install with:
  ```bash
  sudo apt install python3-colcon-common-extensions
  ```
For more information on installing Colcon, please refer to the official [Colcon installation guide](https://colcon.readthedocs.io/en/released/user/installation.html).
  
- **RPI-GPIO**: Library to control Raspberry Pi GPIO channels. Install using:
  ```bash
  sudo apt install python3-rpi.gpio
  ```


### Installation and Setup

1. **Setting Permissions**:
   If you encounter a "RuntimeError: No access to /dev/mem", adjust permissions for GPIO access on your Raspberry Pi:
   ```bash
   sudo chmod 777 /dev/mem
   sudo chmod 777 /dev/gpiomem
   ```
   After adjusting, reboot your Raspberry Pi.

2. **Clone the Repository**:
   Navigate to your home directory, then run:
   ```bash
   mkdir ~/ros2_ws/
   cd ~/ros2_ws
   git clone https://github.com/jhiggason/yahboomg1tank.git .
   ```

3. **Build the ROS2 Package**:
   Compile the project using `colcon`:
   ```bash
   colcon build
   ```

4. **Source the Workspace**:
   After building, source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

5. **Running the Node**:
   To activate and run your ROS2 node, execute:
   ```bash
   ros2 run tank_control_pkg tank_control
   ```


### First Use:
Upon successfully setting up and running the node, the YAHBOOM G1 Tank should now respond to the twist messages provided by the `turtlesim` teleop or any other control method you've implemented.

**Running the Turtlesim Node**
   To activate and run your turtlesim node, execute:
   ```bash
   ros2 run turtlesim turtle_teleop_key --ros-args --remap /turtle1/cmd_vel:=/cmd_vel
   ```

Use your arrow keys on your keyboard to move the tank around.  

---

For any issues or to contribute, engage with this GitHub repository.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!--  How It Works -->
## How It Works:

The `TankControl` class, found in [tank_control.py](https://github.com/jhiggason/yahboomg1tank/blob/main/src/tank_control_pkg/tank_control_pkg/tank_control.py), is designed to control a tank-like robot using ROS2 and Raspberry Pi GPIO pins. It subscribes to ROS2 topics to receive motion commands and controls the robot's movement accordingly.

1. **Initialization**: The class initializes upon instantiation, setting up GPIO pins for the left and right motors and configuring PWM (Pulse Width Modulation) for speed control.

2. **ROS2 Subscriptions**: It subscribes to the `/cmd_vel` topic to receive `Twist` messages, containing linear and angular velocities for the tank's movement.

3. **Motor Speed and Direction Control**: The class processes incoming `Twist` messages to calculate the speeds for the left and right wheels. It applies a correction factor to the linear velocity and amplifies the angular velocity. The speeds are normalized and used to control the motor direction and speed through GPIO pins.

4. **Safety Features**: An inactivity timer is implemented to stop the motors if no command is received for a short duration (currently 0.1 seconds), ensuring safety.

## Important Parameters:

- **GPIO Pins Configuration**: `self.left_motor_pins` and `self.right_motor_pins` define the GPIO pins for the motors.
  
- **Message Processing**: The `subscription_callback` method processes `Twist` messages, applying a correction factor and amplifying the angular velocity to determine motor speeds.

- **Inactivity Timer**: The `timer_callback` method checks for command inactivity, stopping the motors if no command is received within 0.1 seconds.

## Customization:

- **Motor Pins**: Modify `self.left_motor_pins` and `self.right_motor_pins` to match your GPIO configuration.
  
- **Subscription Topic**: Change the topic name in `create_subscription` if needed.
  
- **Control Logic**: Adjust the `subscription_callback` method for different tank motion logic.
  
- **Inactivity Duration**: Alter the inactivity duration in `timer_callback` for different safety requirements.

Careful consideration should be given when modifying GPIO configurations to prevent damage to the Raspberry Pi or connected components.

_For more examples, please refer to the [Documentation](https://github.com/jhiggason/YahBoomG1Tank#getting-started)_

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- Gamepad -->
## Gamepad
In this example we are going to use an Xbox Series X/S controller, connected to bluetooth on the raspberry pi, to send twist MSGs to the /cmd_vel topic.  This will control our tank. 

---

**Setting Up Xbox Controller on Ubuntu 22 for ROS 2*

1. **Disable Secure Boot**:
    If you are doing this on a system other than raspberry pi-please disable secure boot. 
    - Enter your PC's BIOS/UEFI firmware settings during boot, typically by pressing `Del`, `F2`, `F12`, or `Esc`.
    - Navigate to 'Boot' or 'Security'.
    - Find 'Secure Boot' and set it to 'Disabled'.
    - Save and exit.

2. **Update Xbox Controller**:
    * **On Windows**:
        - Connect the controller and open the 'Xbox Accessories' app.
        - Follow on-screen instructions if an update is available.
    * **On Xbox**:
        - Connect the controller, navigate to 'Settings' > 'Devices & connections' > 'Accessories'.
        - Update if prompted.

3. **Install Xbox Driver for Ubuntu**:
    - Install Dependencies:
      ```bash
      sudo apt update
      sudo apt install dkms git
      ```
    - Get `xpadneo` from its repository:
      ```bash
      git clone https://github.com/atar-axis/xpadneo.git
      cd xpadneo
      sudo ./install.sh
      ```

4. **Install Joystick Tools**:
    ```bash
    sudo apt update
    sudo apt install jstest-gtk joystick
    ```

5. **Pair Xbox Controller via Bluetooth**:
    - Start the Bluetooth CLI tool:
      ```bash
      sudo bluetoothctl
      ```
    - Turn on the agent and set it as default:
      ```bash
      agent on
      default-agent
      ```
    - Scan for devices:
      ```bash
      scan on
      ```
    - Turn on your Xbox controller in pairing mode. Note its MAC address.
    - Pair, trust, and connect using the MAC address (replace 'XX:XX:XX:XX:XX:XX'):
      ```bash
      pair XX:XX:XX:XX:XX:XX
      trust XX:XX:XX:XX:XX:XX
      connect XX:XX:XX:XX:XX:XX
      ```
    - Finish:
      ```bash
      scan off
      exit
      ```

6. **Install `ros-humble-teleop-twist-joy`**:
    ```bash
    sudo apt update
    sudo apt install ros-humble-teleop-twist-joy
    ```
7. **Adjust xbox joy settings**:
    ```bash
    cd /opt/ros/humble/share/teleop_twist_joy/config/
    sudo nano xbox.config.yaml
    ```

    Edit the file and change the values to 0 and 1.
   ```bash
    enable_button: 0  # Button A
    enable_turbo_button: 1  # Button B
   ```
   Then Ctrl+O then CTRL+X
8. **Run Teleop Twist joy**
    ```bash
    ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
    ```

Now you should be able to control your tank with the xbox controller after pressing the "A" button on the controller and moving the left stick. 


    

---



<!-- ROADMAP -->
## Roadmap

- [X] Working ROS2 that talks on the correct raspberry pi pins
- [X] Improve movement commands to allow for better control of tank
- [X] Use xbox controller to move the tank
- [X] Enable "search" LEDs and servo http://www.yahboom.net/study/G1-T-PI
- [X] Change to a YAML style config
- [ ] Create proper guide on how to set it up, devolop it, and maintain it
- [ ] Enable camera for remote viewing
- [ ] Enable ultra-sonic sensor and servo
- [ ] Enable basic obstacle avoidance using ultra-sonic sensor
- [ ] Enable advanced obstacle avoidance/path planning via Lidar and Depth Camera

See the [open issues](https://github.com/jhiggason/YahBoomG1Tank/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the MIT License. See [LICENSE](https://github.com/jhiggason/YahBoomG1Tank/blob/main/LICENSE) for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Your Name - Jeff Higgason - jeffh@opensar.net

Project Link: [https://github.com/jhiggason/YahBoomG1Tank](https://github.com/jhiggason/YahBoomG1Tank)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/jhiggason/YahBoomG1Tank.svg?style=for-the-badge
[contributors-url]: https://github.com/jhiggason/YahBoomG1Tank/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/jhiggason/YahBoomG1Tank.svg?style=for-the-badge
[forks-url]: https://github.com/jhiggason/YahBoomG1Tank/network/members
[stars-shield]: https://img.shields.io/github/stars/jhiggason/YahBoomG1Tank.svg?style=for-the-badge
[stars-url]: https://github.com/jhiggason/YahBoomG1Tank/stargazers
[issues-shield]: https://img.shields.io/github/issues/jhiggason/YahBoomG1Tank.svg?style=for-the-badge
[issues-url]: https://github.com/jhiggason/YahBoomG1Tank/issues
[license-shield]: https://img.shields.io/github/license/jhiggason/YahBoomG1Tank.svg?style=for-the-badge
[license-url]: https://github.com/jhiggason/YahBoomG1Tank/blob/master/LICENSE
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/jeff-higgason-2a20a9227/
[Python.org]: https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white
[Python-url]: https://www.python.org/
[ROS.org]: https://img.shields.io/badge/ROS2-Humble-%2322314E?style=for-the-badge&logo=ros&logoColor=%2322314E
[ROS-url]: https://docs.ros.org/en/humble/index.html
