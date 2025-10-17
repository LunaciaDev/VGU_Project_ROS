#!/bin/bash

# Automatic setup script

# Halt the script if any command fails to run
set -e

# Set up ROS Jazzy Repositories
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Install ros-base and rosdep for dependencies management
sudo apt-get update && sudo apt-get install -y python3-rosdep ros-jazzy-ros-base python3-colcon-common-extensions g++
# Setup environment
source /opt/ros/jazzy/setup.bash
# Instantiate rosdep and install all dependencies found
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
# Rollback rviz-common version due to moveit-setup-assistant incompatiability
# See https://github.com/moveit/moveit2/issues/3541
wget --no-check-certificate -O /tmp/ros-rviz.deb https://snapshots.ros.org/jazzy/2025-05-23/ubuntu/pool/main/r/ros-jazzy-rviz-common/ros-jazzy-rviz-common_14.1.11-1noble.20250520.201719_amd64.deb
sudo dpkg -i /tmp/ros-rviz.deb
# Remove downloaded packages - we already installed them.
rm /tmp/ros2-apt-source.deb
rm /tmp/ros-rviz.deb

# Set up X Forwarding for GUI tools
# To use GUI tool in Codespace, the local machine need to have Github CLI installed.
# Then run `gh cs ssh -- -X` to connect to the codespace with X Forwarding
sudo apt-get install -y openssh-server
echo "export DISPLAY='127.0.0.1:10.0'" >> ~/.bashrc
touch ~/.Xauthority

# Automatic sourcing of ROS files for new terminal session
# Will print an error if the workspace was not built
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source /workspaces/VGU_Project_ROS/install/setup.bash" >> ~/.bashrc
echo "export ROS_LOG_DIR='/tmp/ros'" >> ~/.bashrc