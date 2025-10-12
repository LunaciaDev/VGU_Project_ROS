#!/bin/bash

# Automatic setup script

# Halt the script if any command fails to run
set -e

# Set up ROS Jazzy Repositories
sudo apt install software-properties-common
sudo add-apt-repository universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Install ros base, build tools and moveit
sudo apt update && sudo apt install ros-dev-tools ros-jazzy-ros-base ros-jazzy-moveit 
# Rollback rviz-common version due to moveit-setup-assistant incompatiability
# See https://github.com/moveit/moveit2/issues/3541
wget --no-check-certificate -O /tmp/ros-rviz.deb https://snapshots.ros.org/jazzy/2025-05-23/ubuntu/pool/main/r/ros-jazzy-rviz-common/ros-jazzy-rviz-common_14.1.11-1noble.20250520.201719_amd64.deb
sudo dpkg -i /tmp/ros-rviz.deb
rm /tmp/ros2-apt-source.deb
rm /tmp/ros-rviz.deb

# Optionally set up X-Forwarding for GUI tools
sudo apt install openssh-server
touch ~/.Xauthority

# Export automatic ros sourcing
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc