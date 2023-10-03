# YAHBOOM G1 Tank ROS2

![Contributors][contributors-shield]
![Forks][forks-shield]
![Stargazers][stars-shield]
![Issues][issues-shield]
![MIT License][license-shield]
![LinkedIn][linkedin-shield]

<div align="center">
    <a href="https://github.com/jhiggason/YahBoomG1Tank">
        <img src="img/tank.png" alt="Logo" width="670" height="268">
    </a>
</div>

<div align="left">
    
## Overview
A ROS2 Python node designed for the **YAHBOOM G1 Tank** built upon **Raspberry Pi 4**. Initially, it employs the teleop executable from `turtlesim` for foundational control, with plans for advanced methodologies in the pipeline.

**Key Features:**
- Controlled via `turtlesim` teleop executable.
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

The Python node can be found here: [tank_control.py](https://github.com/jhiggason/yahboomg1tank/blob/main/src/tank_control_pkg/tank_control_pkg/tank_control.py)

The `TankControl` class has been designed to control a tank-like robot using the ROS2 framework and a Raspberry Pi's GPIO pins. This class is integrated into the ROS2 ecosystem, allowing it to subscribe to topics and receive messages to control the tank's motion.

1. **Initialization**: Upon instantiation of the `TankControl` class, it initializes the GPIO pins for controlling left and right motors of the tank. Additionally, it sets up the PWM (Pulse Width Modulation) for both motors. PWM is utilized to control the speed of the motors.
   
2. **ROS2 Subscriptions**: The class subscribes to the `/turtle1/cmd_vel` topic, which expects messages of type `Twist`. This `Twist` message essentially contains the linear and angular velocity desired for the tank.
   
3. **Motor Control**: Using the received `Twist` message, the class determines the desired motion (forward, backward, turn left, turn right) and controls the GPIO pins accordingly. The `drive` method sets the appropriate GPIO pins to execute the desired motion at the specified speed.
   
4. **Inactivity Timer**: A timer runs a callback function at a rate of about 10Hz. If no motion command (`Twist` message) is received for 0.15 seconds, the tank's motors are stopped for safety.

## Important Parameters:

- **GPIO Pins Configuration**: The GPIO pins for controlling the left and right motors are defined with `self.left_motor_pins` and `self.right_motor_pins`. They contain the pins for Forward, Reverse, and PWM control.

- **Subscription Topic and Message**: The class subscribes to the `/turtle1/cmd_vel` topic expecting `Twist` messages to derive the motion control instructions.

- **Motor Control**: The motor control and its directionality are derived from the linear and angular components of the received `Twist` message.

## Customization:

To adapt the code to your specific requirements or setup:

1. **Motor Pins**: Modify `self.left_motor_pins` and `self.right_motor_pins` if your setup uses different GPIO pins or if you wish to change the pin assignments.

2. **Subscription Topic**: If your control source publishes to a different topic or if you want to change the topic name, update the topic in the `create_subscription` method.

3. **Control Logic**: If your tank's motion logic differs from the provided code, the necessary changes should be made in the `subscription_callback` method, which processes the received `Twist` messages and controls the tank.

4. **Inactivity Duration**: The threshold for motor inactivity (currently 0.15 seconds) can be modified in the `timer_callback` method. 

Remember, any changes to the GPIO configuration or usage should be made with care, ensuring the Raspberry Pi and any connected components are protected from incorrect configurations or potential damage.

_For more examples, please refer to the [Documentation](https://github.com/jhiggason/YahBoomG1Tank#getting-started)_

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ROADMAP -->
## Roadmap

- [X] Working ROS2 that talks on the correct raspberry pi pins
- [X] Improve movement commands to allow for better control of tank
- [ ] Use xbox controller to move the tank

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
