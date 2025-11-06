#include "unity_targets_listener.hpp"

#include "joint_debug.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/utils/moveit_error_code.h"
#include "ros/console.h"
#include "ros/duration.h"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using UnityRequest = ros_unity_messages::UnityRequest;

void unity_targets_subs_handler(const UnityRequest::ConstPtr& message) {
    static const std::string ARM_PLANNING_GROUP = "robot_arm";
    static const std::string GRIPPER_PLANNING_GROUP = "robot_gripper";
    static const ros::Duration sleep_timer = ros::Duration(1, 500000);
    ROS_INFO("Received target message from Unity.");

    MoveGroupInterface arm_move_group_interface(ARM_PLANNING_GROUP);
    MoveGroupInterface gripper_move_group_interface(GRIPPER_PLANNING_GROUP);

    // Allow replanning if scene change, would come in useful when scene change?
    arm_move_group_interface.allowReplanning(true);
    // Allow replan attempt in case the planner simply didnt find a path, there are time when it does that
    arm_move_group_interface.setNumPlanningAttempts(5);

    // Same config for gripper
    gripper_move_group_interface.setNumPlanningAttempts(5);

    // Uncomment if you want to check Unity joint control script
    // debug_joint(move_group_interface);

    // Let's start plan and execute!
    // Pre-Grasp pose
    ROS_INFO("Planning and executing pre-grasp pose");
    arm_move_group_interface.setPoseTarget(message->pick_pose, "arm_tcp_link");
    if (arm_move_group_interface.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("Failed to move to pre_grasp pose, exiting");
        return;
    }
    sleep_timer.sleep();
    ROS_INFO("Pre-grasp pose executed");

    // Open the gripper
    gripper_move_group_interface.setNamedTarget("gripper_open");
    gripper_move_group_interface.move();

    // Pick pose
    {
        auto pick_pose = geometry_msgs::Pose(message->pick_pose);
        pick_pose.position.z = pick_pose.position.z - 0.18;

        ROS_INFO("Planning and executing pick pose");
        arm_move_group_interface.setPoseTarget(pick_pose, "arm_tcp_link");
        if (arm_move_group_interface.move() !=
            moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Failed to move to pick pose, exiting");
            return;
        }
    }
    sleep_timer.sleep();
    ROS_INFO("Pick pose executed");

    // Close the gripper
    gripper_move_group_interface.setNamedTarget("gripper_close");
    gripper_move_group_interface.move();

    // Pickup Pose
    ROS_INFO("Planning and executing pickup pose");
    arm_move_group_interface.setPoseTarget(message->pick_pose, "arm_tcp_link");
    if (arm_move_group_interface.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("Failed to move to pickup pose, exiting");
        return;
    }
    sleep_timer.sleep();
    ROS_INFO("Pickup pose executed");

    // Pre-place Pose
    ROS_INFO("Planning and executing pre-place pose");
    arm_move_group_interface.setPoseTarget(message->place_pose, "arm_tcp_link");
    if (arm_move_group_interface.move() != moveit::core::MoveItErrorCode::SUCCESS) {
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
        arm_move_group_interface.setPoseTarget(place_pose, "arm_tcp_link");
        if (arm_move_group_interface.move() !=
            moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Failed to move to place pose, exiting");
            return;
        }
    }
    sleep_timer.sleep();
    ROS_INFO("Place pose executed");

    // Open the gripper
    gripper_move_group_interface.setNamedTarget("gripper_open");
    gripper_move_group_interface.move();

    // Lift-up Pose
    ROS_INFO("Planning and executing lift-up pose");
    arm_move_group_interface.setPoseTarget(message->place_pose, "arm_tcp_link");
    if (arm_move_group_interface.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("Failed to move to lift-up pose, exiting");
        return;
    }
    sleep_timer.sleep();
    ROS_INFO("Lift-up pose executed");

    // Return gripper to neutral
    gripper_move_group_interface.setNamedTarget("gripper_neutral");
    gripper_move_group_interface.move();

    // Return to starting position
    {
        std::vector<double> joint_group_position;
        joint_group_position.resize(6, 0);
        arm_move_group_interface.setJointValueTarget(joint_group_position);
        if (arm_move_group_interface.move() !=
            moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Failed to move to all-zero pose, exiting");
            return;
        }
    }
    ROS_INFO("All-zero pose executed");
}