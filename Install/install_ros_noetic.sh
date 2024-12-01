#!/bin/bash

# This script installs ROS Noetic Desktop Full on Ubuntu

# Function to print messages
print_message() {
    echo "====================================================================="
    echo "$1"
    echo "====================================================================="
}

# Check if the user is root
if [ "$EUID" -eq 0 ]; then
    echo "Please run this script as a regular user, not as root (do not use sudo)."
    exit 1
fi

# Check Ubuntu version
print_message "Checking Ubuntu version..."
UBUNTU_VERSION=$(lsb_release -sc)
if [[ "$UBUNTU_VERSION" != "focal" ]]; then
    echo "This script only supports Ubuntu 20.04 (Focal)."
    exit 1
fi

# Update package list
print_message "Updating package list..."
sudo apt update

# Install prerequisites
print_message "Installing prerequisites..."
sudo apt install -y curl gnupg2 lsb-release

# Add ROS Noetic APT source
print_message "Adding ROS Noetic APT source..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Add ROS key
print_message "Adding ROS package key..."
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Update package list again
print_message "Updating package list with ROS source..."
sudo apt update

# Install ROS Noetic Desktop Full
print_message "Installing ROS Noetic Desktop Full..."
sudo apt install -y ros-noetic-desktop-full

# Initialize ROS dependencies
print_message "Initializing rosdep..."
sudo rosdep init
rosdep update

# Configure environment
print_message "Configuring ROS environment..."
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install additional ROS tools
print_message "Installing additional ROS tools..."
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Test ROS installation
print_message "Testing ROS installation..."
if command -v roscore >/dev/null 2>&1; then
    echo "ROS Noetic Desktop Full installation completed successfully!"
    echo "You can start ROS with the command: roscore"
else
    echo "ROS installation failed. Please check the output above for errors."
fi
