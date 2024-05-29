#!/bin/bash

# Add the ROS package repository to your sources list
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Install curl if it's not already installed
apt install -y curl

# Add the ROS package signing key
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Update package index
apt update

# Install ROS Noetic full desktop version
apt install -y ros-noetic-desktop-full

# Search for ROS Noetic packages (optional)
apt search ros-noetic

# Add ROS setup script to bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Source the bashrc to update current shell session
source ~/.bashrc

# Install additional ROS dependencies
apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
rosdep init

# Update rosdep
rosdep update

# Install shapely with pip
pip install shapely

# Install requirements for the perception package
pip install -r requirements.txt
