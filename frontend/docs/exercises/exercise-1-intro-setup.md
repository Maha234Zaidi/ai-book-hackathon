---
title: Exercise 1 - ROS 2 Environment Setup
description: Hands-on exercise for setting up ROS 2 environment
sidebar_position: 1
---

# Exercise 1: ROS 2 Environment Setup

## Objective

Successfully install ROS 2 Humble Hawksbill on your system and verify the installation with basic commands.

## Prerequisites

- A computer running Ubuntu 22.04 or Windows 10/11
- Administrative privileges to install software
- Stable internet connection
- Basic familiarity with command line interface

## Steps

### 1. System Preparation

1. Open a terminal (Ubuntu) or PowerShell/Command Prompt (Windows)
2. Ensure your system is up to date:
   - On Ubuntu: `sudo apt update && sudo apt upgrade`
   - On Windows: Ensure you're running as Administrator

### 2. Install ROS 2 Humble Hawksbill

Follow the official installation guide for your operating system:

#### Ubuntu 22.04:
1. Set locale:
   ```bash
   locale  # check for 'C.UTF-8' or 'en_US.UTF-8' locale
   sudo locale-gen en_US.UTF-8
   sudo update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
   export LANG=en_US.UTF-8
   ```

2. Add the ROS 2 apt repository:
   ```bash
   sudo apt update && sudo apt install curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

3. Install ROS 2 packages:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

4. Install colcon build tools:
   ```bash
   sudo apt install python3-colcon-common-extensions
   ```

#### Windows 10/11:
1. Install Chocolatey package manager
2. Install Open Robotics Signing Key:
   ```cmd
   choco sources add -n=ros-win -s https://aka.ms/ros/public --priority=1
   ```

3. Install ROS 2 Humble Hawksbill:
   ```cmd
   choco install ros-humble-desktop
   ```

4. Install colcon build tools:
   ```cmd
   pip install colcon-common-extensions
   ```

### 3. Source the ROS 2 Environment

1. On Ubuntu, add the following line to your `~/.bashrc`:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. On Windows, run:
   ```cmd
   "c:\dev\ros_humble\setup.bat"
   ```

3. Either restart your terminal or source the environment in the current session:
   ```bash
   source /opt/ros/humble/setup.bash  # On Ubuntu
   # Or in PowerShell on Windows:
   # "c:\dev\ros_humble\setup.bat"
   ```

### 4. Verify Installation

1. Check if ROS 2 commands are available:
   ```bash
   ros2 --version
   ```

2. Verify environment variables are set:
   ```bash
   printenv | grep -i ros
   ```

3. Create a test workspace:
   ```bash
   mkdir -p ~/ros2_test_ws/src
   cd ~/ros2_test_ws
   colcon build
   source install/setup.bash
   ```

4. Check available nodes, topics, and services:
   ```bash
   ros2 node list
   ros2 topic list
   ros2 service list
   ```

### 5. Run Basic Demo

1. In one terminal, run a simple publisher:
   ```bash
   ros2 run demo_nodes_cpp talker
   ```

2. In another terminal, run a simple subscriber:
   ```bash
   ros2 run demo_nodes_py listener
   ```

3. You should see messages being published and received between nodes.

## Expected Result

- ROS 2 Humble Hawksbill successfully installed on your system
- All verification commands execute without errors
- Basic talker/listener demo runs and displays communication between nodes
- Environment variables properly set for ROS 2 usage

## Troubleshooting

- If `ros2 --version` command is not found, ensure your environment is properly sourced
- If talker/listener demo doesn't work, check that both terminals have the ROS 2 environment sourced
- For Windows installation issues, ensure you're running with Administrator privileges
- If you encounter locale issues, ensure the C.UTF-8 or en_US.UTF-8 locale is properly set

## Learning Outcome

After completing this exercise, you should have a functional ROS 2 environment on your system and understand the basics of sourcing the ROS 2 environment. This foundational setup is essential for all subsequent ROS 2 development work.