# Development Environment Setup Guide

This guide provides instructions for setting up the development environment required for Module 2: The Digital Twin (Gazebo & Unity). You'll need to install several tools and frameworks to properly implement and test the concepts covered in this module.

## Prerequisites

Before beginning the setup, ensure you have:
- A computer running Ubuntu 22.04 LTS (or equivalent ROS 2 environment)
- Administrator access to install software
- At least 20GB of free disk space
- 8GB or more of RAM (recommended for Unity development)

## Required Software

The following software packages are required for this module:

1. **ROS 2 Humble Hawksbill** - Robot Operating System for communication and control
2. **Gazebo Garden** - Physics simulation environment
3. **Unity 2022.3 LTS** - Visualization and rendering engine
4. **Python 3.10** - Scripting and development
5. **Git** - Version control

## Installing ROS 2 Humble Hawksbill

### 1. Set locale settings
```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 2. Add ROS 2 GPG key and repository
```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros-keyring.gpg -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 3. Install ROS 2
```bash
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-argcomplete
sudo apt install python3-colcon-common-extensions
```

### 4. Source ROS 2 environment
```bash
source /opt/ros/humble/setup.bash
```

To automatically source ROS 2 when opening a new terminal, add this line to your `.bashrc`:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Installing Gazebo Garden

### 1. Add Gazebo repository
```bash
sudo apt install software-properties-common lsb-release
sudo add-apt-repository "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main"
sudo apt update
sudo apt install gazebo
```

### 2. Verify installation
```bash
gazebo --version
```

## Installing Unity 2022.3 LTS

Unity installation requires a few more steps:

1. **Download Unity Hub**:
   - Visit https://unity.com/download
   - Download Unity Hub for your operating system
   - Install Unity Hub following the platform-specific instructions

2. **Install Unity 2022.3 LTS through Unity Hub**:
   - Open Unity Hub
   - Go to the "Installs" tab
   - Click "Add" to install a new Unity version
   - Select "2022.3.20f1 LTS" (or the latest LTS version in the 2022.3 series)
   - Select the modules you need (Linux Build Support is recommended)

3. **Install Unity Robotics SDK**:
   - Open Unity Hub and create a new 3D project
   - In Unity, go to Window → Package Manager
   - Click the + icon in the top left and select "Add package from git URL..."
   - Add the Unity Robotics package repository

## Environment Verification

After installation, verify that all components are properly installed:

### 1. Check ROS 2 installation
```bash
ros2 --version
```

### 2. Test Gazebo
```bash
gazebo
```

### 3. Test Unity
- Open Unity Hub
- Create a new 3D project to verify Unity runs correctly

## Troubleshooting Common Issues

### ROS 2 Installation Issues
- If you encounter issues with the ROS 2 repository, verify your Ubuntu version matches the supported versions.
- Make sure your system time is synchronized with an NTP server.

### Gazebo Not Launching
- Ensure your graphics drivers are up to date
- Check that you have the necessary graphics libraries installed:
  ```bash
  sudo apt install nvidia-prime intel-media-va-driver mesa-utils
  ```

### Unity Installation on Linux
- Unity officially supports Linux through IL2CPP build support
- For full editor functionality, you may need to run Unity on Windows or macOS
- Consider using Unity Cloud Build for Linux deployments

## Next Steps

After completing the environment setup, you can proceed with the hands-on exercises in each section of this module. Each section includes specific examples and exercises that build upon the installed tools.

Remember that this setup needs to be done on the development machine where you'll be implementing and testing the digital twin examples in this module.