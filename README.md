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
- [Usage](#usage)
- [Roadmap](#roadmap)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## About The Project

![Project Image](https://hackster.imgix.net/uploads/attachments/1418126/_y4NHHNkT2t.blob?auto=compress%2Cformat&w=900&h=675&fit=min)

### Built With
- [Python3][Python-url]
- [ROS2 Humble][ROS-url]

## Getting Started
A step-by-step guide on setting up the ROS2 node for YAHBOOM G1 Tank with Raspberry Pi 4.

### Prerequisites

**Hardware:**
- YAHBOOM G1 Tank (fully assembled)
- Raspberry Pi 4 (interfaced with G1 Tank)

**Software:**
- Ubuntu 22 for Raspberry Pi [Installation Guide](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#1-overview)
- ROS2 Humble-desktop [Installation Guide](https://index.ros.org/doc/ros2/Installation/Humble/)
- Colcon [Installation Guide](https://colcon.readthedocs.io/en/released/user/installation.html)
- RPI-GPIO

### Installation and Setup

1. **Permissions Adjustments**
2. **Repository Cloning**
3. **ROS2 Package Building**
4. **Workspace Sourcing**
5. **Node Activation**

### First Use

Instructions on utilizing `turtlesim` teleop and associated features.

## Usage

An overview of the TankControl class, its setup, and the methodologies it employs.

## Roadmap

- ✅ Working ROS2 pin communications
- ⌛ Improved movement commands
- 🔜 Xbox controller integration

[View Open Issues](https://github.com/jhiggason/YahBoomG1Tank/issues)

## Contributing

Guidelines for contributing to the project.

## License

Licensed under MIT. [View License](https://github.com/jhiggason/YahBoomG1Tank/blob/main/LICENSE).

## Contact

Jeff Higgason - jeffh@opensar.net  
[Project Link](https://github.com/jhiggason/YahBoomG1Tank)

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
