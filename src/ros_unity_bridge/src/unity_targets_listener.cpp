#include "unity_targets_listener.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/utils/moveit_error_code.h"
#include "joint_debug.hpp"
#include "ros/console.h"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using UnityRequest = ros_unity_messages::UnityRequest;

void unity_targets_subs_handler(const UnityRequest::ConstPtr& message) {
    static const std::string PLANNING_GROUP = "robot_arm";
    ROS_INFO("Received target message from Unity.");

    MoveGroupInterface move_group_interface(PLANNING_GROUP);

    // Uncomment if you want to check Unity joint control script
    // debug_joint(move_group_interface);

    // Let's start plan and execute!
    // Pre-Grasp pose
    ROS_INFO("Planning and executing pre-grasp pose");
    move_group_interface.setPoseTarget(message->pick_pose, "arm_tcp_link");
    if (move_group_interface.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("Failed to move to pre_grasp pose, exiting");
        return;
    }
    ROS_INFO("Pre-grasp pose executed");

    // Pick pose
    {
        auto pick_pose = geometry_msgs::Pose(message->pick_pose);
        pick_pose.position.z = pick_pose.position.z - 0.05;

        ROS_INFO("Planning and executing pick pose");
        move_group_interface.setPoseTarget(pick_pose, "arm_tcp_link");
        if (move_group_interface.move() !=
            moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Failed to move to pick pose, exiting");
            return;
        }
    }
    ROS_INFO("Pick pose executed");

    // Pickup Pose
    ROS_INFO("Planning and executing pickup pose");
    move_group_interface.setPoseTarget(message->pick_pose, "arm_tcp_link");
    if (move_group_interface.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("Failed to move to pickup pose, exiting");
        return;
    }
    ROS_INFO("Pickup pose executed");

    // Place Pose
    ROS_INFO("Planning and executing place pose");
    move_group_interface.setPoseTarget(message->place_pose, "arm_tcp_link");
    if (move_group_interface.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("Failed to move to pre_grasp pose, exiting");
        return;
    }
    ROS_INFO("Place pose executed");
}