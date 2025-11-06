#include "unity_targets_listener.hpp"

#include "joint_debug.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/utils/moveit_error_code.h"
#include "ros/console.h"
#include "ros/duration.h"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using UnityRequest = ros_unity_messages::UnityRequest;

void unity_targets_subs_handler(const UnityRequest::ConstPtr& message) {
    static const std::string PLANNING_GROUP = "robot_arm";
    static const ros::Duration sleep_timer = ros::Duration(2, 0);
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
    sleep_timer.sleep();
    ROS_INFO("Pre-grasp pose executed");

    // Pick pose
    {
        auto pick_pose = geometry_msgs::Pose(message->pick_pose);
        pick_pose.position.z = pick_pose.position.z - 0.15;

        ROS_INFO("Planning and executing pick pose");
        move_group_interface.setPoseTarget(pick_pose, "arm_tcp_link");
        if (move_group_interface.move() !=
            moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Failed to move to pick pose, exiting");
            return;
        }
    }
    sleep_timer.sleep();
    ROS_INFO("Pick pose executed");

    // Pickup Pose
    ROS_INFO("Planning and executing pickup pose");
    move_group_interface.setPoseTarget(message->pick_pose, "arm_tcp_link");
    if (move_group_interface.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("Failed to move to pickup pose, exiting");
        return;
    }
    sleep_timer.sleep();
    ROS_INFO("Pickup pose executed");

    // Pre-place Pose
    ROS_INFO("Planning and executing pre-place pose");
    move_group_interface.setPoseTarget(message->place_pose, "arm_tcp_link");
    if (move_group_interface.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("Failed to move to pre_grasp pose, exiting");
        return;
    }
    sleep_timer.sleep();
    ROS_INFO("Pre-place pose executed");

    // Place Pose
    {
        auto place_pose = geometry_msgs::Pose(message->place_pose);
        place_pose.position.z = place_pose.position.z - 0.15;
        ROS_INFO("Planning and executing place pose");
        move_group_interface.setPoseTarget(place_pose, "arm_tcp_link");
        if (move_group_interface.move() !=
            moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Failed to move to place pose, exiting");
            return;
        }
    }
    sleep_timer.sleep();
    ROS_INFO("Place pose executed");

    // Lift-up Pose
    ROS_INFO("Planning and executing lift-up pose");
    move_group_interface.setPoseTarget(message->place_pose, "arm_tcp_link");
    if (move_group_interface.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("Failed to move to lift-up pose, exiting");
        return;
    }
    sleep_timer.sleep();
    ROS_INFO("Lift-up pose executed");

    // Return to starting position
    {
        std::vector<double> joint_group_position;
        joint_group_position.resize(6, 0);
        move_group_interface.setJointValueTarget(joint_group_position);
        if (move_group_interface.move() !=
            moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Failed to move to all-zero pose, exiting");
            return;
        }
    }
    ROS_INFO("All-zero pose executed");
}