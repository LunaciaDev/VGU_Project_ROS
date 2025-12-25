# VGU_Project_ROS

This repository is part of module 61CSE126 "Project" in Vietnamese-German Univeristy, instructed by Dr.-Ing. Quang Huan Dong ([huan.dq@vgu.edu.vn](mailto:huan.dq@vgu.edu.vn)).

The goal of the project is to design a planning system that take control of the [UR10e](https://www.universal-robots.com/products/ur10e/) robot mounted with the [RG2](https://onrobot.com/en/products/rg2-finger-gripper) gripper in order to execute pick and place tasks in both static and dynamic environment. During development, the robot's environment is simulated in [Unity](https://unity.com/), while the robot itself is simulated by [URSim](https://www.universal-robots.com/download/software-e-series/simulator-non-linux/offline-simulator-e-series-ur-sim-for-non-linux-5126-lts/).

The repositories that are part of the project are listed below:

- ROS: [https://github.com/LunaciaDev/VGU_Project_ROS](https://github.com/LunaciaDev/VGU_Project_ROS)
- Unity Simulation: [https://github.com/LunaciaDev/VGU_Project_Unity](https://github.com/LunaciaDev/VGU_Project_Unity)

# Warning

While the software can support being connected into an actual robot, please make sure that you have calibrated your robot and regenerated the description based on that calibration! The description used here is *generic* and may causes issues during execution.

# Overview

This repository holds the Robot Operating System ([ROS](https://www.ros.org/)) components used to control the robot. We targets the [Noetic Ninjemys](https://wiki.ros.org/noetic) release.

## Structure

```
src
|- dependencies
|  |- onrobot_rg_description
|  |- ros_tcp_endpoint
|  |- ur_description
|- ros_unity_bridge
|- ros_unity_messages
|- ur10e_config
   |- ur10e_description
   |- ur10e_moveit
```

`dependencies` holds packages that are needed to be built from source, or package modified to remove out unneeded depdencies.

`dependencies/onrobot_rg_description` (Upstream repository [here](https://github.com/UOsaka-Harada-Laboratory/onrobot)) package provide the description for Onrobot RG2 Gripper.

`dependencies/ros_tcp_endpoint` (Upstream repository [here](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)) package provide the TCP connection between ROS and Unity.

`dependencies/ur_description` (Upstream repository [here](https://github.com/ros-industrial/universal_robot)) package provide the description for the UR10e Robot.

`ros_unity_bridge` package holds the application and communication code between the simulator in Unity and MoveIt! planner.

`ros_unity_messages` defines the custom messages transmitted between ROS and Unity.

`ur10e_config/ur10e_description` package provide the URDF description for the UR10e robot mounted with the RG2 gripper.

`ur10e_config/ur10e_moveit` package provide the MoveIt! configuration to plan and control the robot.

## Installation

The repository is designed to run inside Github Codespaces, but we also provided a general installation script in [`install.sh`](install.sh). Please note that the general installation script is meant for Ubuntu [Focal Fossa](https://www.releases.ubuntu.com/focal/) release, and may need some tweaks depending on your system and your needs.

## Building

Please make sure that all dependencies are installed (See Installation above) before building.

To build the project, run `catkin_make`.
To also export `compile_commands.json` for development tools, run `catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1`.

## Usage

After building, you will need to launch MoveIt! backend, the communication endpoint between ROS and Unity as well as application code:

```
roslaunch ros_unity_bridge moveit_backend.launch
roslaunch ros_unity_bridge unity_bridge.launch
```

The endpoint is exposed on port 10000 by default.
