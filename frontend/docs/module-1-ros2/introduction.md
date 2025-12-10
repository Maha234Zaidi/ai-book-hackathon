---
title: Introduction to ROS 2
description: Overview and setup guide for ROS 2 framework
sidebar_position: 1
---

# Introduction to ROS 2

## Learning Objectives

- Understand what ROS 2 is and why it's important in robotics
- Explain the key architectural concepts of ROS 2
- Install ROS 2 on your system
- Set up your development environment

## Overview of ROS 2

The Robot Operating System 2 (ROS 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

ROS 2 is not an operating system, but rather a middleware that provides services designed for a heterogeneous computer cluster such as hardware abstraction, device drivers, implementation of commonly used functionality, message-passing between processes, and package management.

## Importance in Robotics

ROS 2 plays a critical role in robotics development by:

1. Providing standardized tools and interfaces
2. Enabling code reusability across different robotic platforms
3. Facilitating collaboration and sharing in the robotics community
4. Supporting both simulation and real-world robot deployment
5. Offering a rich ecosystem of packages and tools

## Architecture Overview

ROS 2 architecture consists of several key components:

- **Nodes**: Processes that perform computation
- **Topics**: Named buses over which nodes exchange messages
- **Services**: Synchronous request/response communication
- **Actions**: Asynchronous goal-oriented communication
- **Parameters**: Configuration values that can be set at runtime
- **Lifecycle nodes**: Nodes with well-defined state transitions

The architecture is designed with quality of service (QoS) settings to allow for real-time and distributed systems.

## Installation Guide

### Prerequisites

Before installing ROS 2, ensure your system meets these requirements:

- Ubuntu 22.04 (Jammy Jellyfish) or Windows 10/11
- Python 3.10
- Python 3.10 development headers
- At least 5GB of free disk space
- A system with Ubuntu packages repository access or equivalent

### Installing ROS 2 Humble Hawksbill

ROS 2 Humble Hawksbill is the recommended long-term support (LTS) distribution for this course. It provides stability and long-term support until 2027.

#### On Ubuntu 22.04:

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

5. Source the ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

#### On Windows 10/11:

1. Install Chocolatey package manager
2. Install Open Robotics Signing Key:
   ```cmd
   choco sources add -n=ros-win -s https://aka.ms/ros/public --priority=1
   ```

3. Install ROS 2 Humble Hawksbill:
   ```cmd
   choco install ros-humble-desktop
   ```

4. Add ROS 2 to PATH:
   ```cmd
   set PATH=%PATH%;C:\opt\ros\humble\x64;C:\opt\ros\humble\Lib\site-packages
   ```

5. Install colcon build tools:
   ```cmd
   pip install colcon-common-extensions
   ```

## Setting up Your Development Environment

After installation, it's important to set up a proper development workspace:

1. Create a workspace directory:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```

2. Source the ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash  # On Linux
   # Or in PowerShell on Windows:
   # "c:\dev\ros_humble\setup.bat"
   ```

3. Build the workspace:
   ```bash
   colcon build
   ```

4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Summary

This introduction provided you with an overview of ROS 2, its importance in robotics, architectural concepts, and installation steps. With your environment set up, you're now ready to explore more advanced concepts in the following sections.

In the next section, we'll dive into the core communication mechanisms: nodes, topics, and services.