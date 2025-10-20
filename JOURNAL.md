# Dev Journal

Recording of how we worked on this project.

## Top-level TODO

- [ ] Request lab time to get calibration data

## 20 Oct 2025

- [x] Write the description for the ur10e mounted with rg2
- [x] Create the moveit package
- [ ] Migrate the message and service definition
- [ ] Migrate the planner

The implementation needed to be moved back to ROS 1. Back to XML launch files.
The configuration has to be redone as the ur10e description changed between ROS 1 and ROS 2 package.
This time, not going to include the world link in the base urdf description.

The moveit package is generated via moveit_config_assistant.
We still need the calibration data for the actual robot.