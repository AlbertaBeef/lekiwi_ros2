# LeKiwi ROS2 - Mobile Base

A ROS2 implementation for the LeKiwi robotic base, providing complete integration with LeRobot framework including hardware interface.

## 🤖 Overview

This repository contains ROS2 packages for controlling the LeKiwi mobile base. It includes:

- **Robot Description**: URDF/XACRO models with 3D meshes
- **Hardware Interface**: Direct motor control via Feetech servos

## 📦 Packages

### `lekiwi_description`
Robot description package containing:
- URDF/XACRO robot model
- 3D mesh files (.stl/.part)
- RViz visualization configuration
- Launch files for robot display

### `lekiwi_hw_interface`
Hardware interface package providing:
- Motor control via Feetech protocol
- Calibration utilities
- Hardware abstraction layer
- LeRobot integration utilities

## 🚀 Quick Start

### Prerequisites
- ROS2 Jazzy
- Python 3.12+

### Installation

1. Clone the repository:
```bash
git clone https://github.com/AlbertaBeef/lekiwi_ros2.git
cd lekiwi_ros2
```

2. Install dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace:
```bash
colcon build
source install/setup.bash
```

### Usage

#### Visualize the robot:
```bash
ros2 launch lekiwi_description display_rviz2.launch.py
```

#### Start hardware interface:
```bash
ros2 run lekiwi_hw_interface lekiwi_motor_bridge
```

## 🎛️ Advanced Usage

### Hardware Control with Joint State Publisher
For manual joint control and visualization with the hardware interface:

```bash
# Terminal 1: Start the motor bridge (hardware interface)
ros2 run lekiwi_hw_interface lekiwi_motor_bridge

# Terminal 2: Launch robot visualization without GUI
ros2 launch lekiwi_description display)_rviz2.launch.py

# Terminal 3: Launch ROS2 node(s) that publish Twist topic (/cmd_vel)

```

This setup allows you to:
- **Visualize** the robot in RViz with real joint states from hardware
- **Monitor** real-time feedback from the Feetech servos

### Topic Remapping Guide
- `/joint_states`: Real joint positions from hardware
- `/joint_commands`: Desired joint positions to hardware
- Use topic remapping (`-r`) to connect different components

## 🔧 Configuration

### Hardware Setup
- Connect Feetech servos via serial/USB interface
- Update motor IDs in `config/lekiwi_calibration.yaml`
- Verify communication with `lekiwi_motor_bridge`

## 📚 Documentation

- [Hardware Setup Guide](docs/hardware_setup.md)
- [Calibration Process](docs/calibration.md)
- [API Reference](docs/api_reference.md)

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgement

This repo is inspired by the following, and is meant to be used with:
- [SO-101 ROS2 Support](https://github.com/AgRoboticsResearch/Lerobot_ros2.git) - Inspiration for this repo

## 🙋‍♂️ Support

For questions and support:
- Open an issue on GitHub
- Check the documentation
- Join our community discussions

## 🔗 Related Projects

- [LeRobot](https://github.com/huggingface/lerobot) - Main LeRobot framework
- [SO-101 Hardware](https://github.com/TheRobotStudio/SO-ARM101) - Original hardware design
- [SO-101 ROS2 Support](https://github.com/AgRoboticsResearch/Lerobot_ros2.git) - Inspiration for this repo

---

**Made with ❤️ for the robotics community** 
