#!/bin/bash

# Halt the script if any step return non-zero
set -euo pipefail

# Enable Restricted, Universe and Multiverse release stream.
sudo apt-get install software-properties-common
sudo add-apt-repository "deb http://archive.ubuntu.com/ubuntu $(lsb_release -sc) main universe restricted multiverse"

# Add ROS Noetic Package Repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
## Import signing key for ROS Noetic packages
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Update the repository information
sudo apt-get update

# Install Base ROS Noetic, ROS dependencies manager, GCC, aliasing `python` to python3, jq to concatenate compile_commands
sudo apt-get install -y ros-noetic-ros-base python3-rosdep g++ python-is-python3 jq python3-catkin-tools mongodb

# Source the environment for ROS
source /opt/ros/noetic/setup.bash

# Install all listed dependencies in package.xml
sudo rosdep init
rosdep update --rosdistro=noetic
rosdep install --from-paths src -y --ignore-src --rosdistro=noetic

## ENVIRONMENT VARIABLES
# Enable/disable them as you see fit, none of these are required

# Automatic sourcing of ROS files for new terminal session
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
# Replace <path> with the local path to this
# echo "source <path>/devel_isolated/setup.bash" >> ~/.bashrc

# Switch to non-persistent logs
echo "export ROS_LOG_DIR='/tmp/ros'" >> ~/.bashrc

# More useful console logs format
echo export ROSCONSOLE_FORMAT=\''[${severity}] [${walltime:%Y-%m-%d %H:%M:%S}] [${node}]: ${message}'\' >> ~/.bashrc

# Disable EOL Warning for RViz
echo "export DISABLE_ROS1_EOL_WARNINGS=1" >> ~/.bashrc