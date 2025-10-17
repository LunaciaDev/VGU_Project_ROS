#!/bin/bash

# Halt the script if any step return non-zero
set -e

# Add ROS Noetic repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
## Add the signing key of Noetic
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Update the repository information
sudo apt-get update

# Install ROS Noetic base and rosdep
sudo apt-get install -y ros-noetic-ros-base python3-rosdep g++

# Install listed dependencies
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src

# Optional: Allow X Forwarding for GUI applications
sudo apt-get install -y openssh-server
echo "export DISPLAY='127.0.0.1:10.0'" >> ~/.bashrc
touch ~/.Xauthority

# Automatic sourcing of ROS files for new terminal session
# Will print an error if the workspace was not built
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /workspaces/VGU_Project_ROS/install/setup.bash" >> ~/.bashrc
echo "export ROS_LOG_DIR='/tmp/ros'" >> ~/.bashrc