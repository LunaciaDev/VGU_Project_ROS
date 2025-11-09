# robot_description

This package provides the URDF description for the UR10e Robot mounted with the RG2 Gripper.

# Overview

The robot is defined in [ur10e_rg2.xacro](urdf/ur10e_rg2.xacro).

After making changes, build the urdf by running `xacro src/ur10e_config/robot_description/urdf/ur10e_rg2.xacro -o src/ur10e_config/robot_description/urdf/ur10e_rg2.urdf`.

## Methodology

The gripper is attached to the robot arm with a fixed joint connecting `tool0` link of the robot arm with `onrobot_rg2_base_link`.

We also added a virtual link `tcp_link` connected to `tool0` to extend the end effector of the arm to the working area of the gripper.