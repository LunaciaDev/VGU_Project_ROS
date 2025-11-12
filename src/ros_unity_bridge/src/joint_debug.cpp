#include "joint_debug.hpp"

#include <cmath>
#include <vector>

#include "ros/console.h"

/**
 * Debug joint configuration by actuating the joint by 1/4th of a turn, then
 * back, for every joints in the robot arm.
 */
void debug_joint(MoveGroupInterface& move_group_interface) {
    // Move each joint and then reset back to 0 for visual check
    const moveit::core::JointModelGroup* joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup("robot_arm");
    const auto tau = 2 * M_PI;

    const auto current_joint_state = move_group_interface.getCurrentState();
    std::vector<double> joint_group_position;
    current_joint_state->copyJointGroupPositions(
        joint_model_group, joint_group_position
    );

    // For each joint, reset the previous joint target and set the new joint
    // target to be 1/4 forward turn in radians then roll back The UR10e is a 6
    // DOF arm.
    for (int i = 0; i < 6; i++) {
        ROS_INFO("Moving joint index %i", i);
        joint_group_position[i] = tau / 4;
        move_group_interface.setJointValueTarget(joint_group_position);
        move_group_interface.move();
        joint_group_position[i] = 0;
        move_group_interface.setJointValueTarget(joint_group_position);
        move_group_interface.move();
    }
}