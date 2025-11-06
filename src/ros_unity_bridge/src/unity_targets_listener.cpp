#include "unity_targets_listener.hpp"

#include "joint_debug.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/utils/moveit_error_code.h"
#include "ros/console.h"
#include "ros/duration.h"
#include "ros_unity_messages/UnityObject.h"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using PlanningSceneInterface =
    moveit::planning_interface::PlanningSceneInterface;
using UnityRequest = ros_unity_messages::UnityRequest;
using UnityObject = ros_unity_messages::UnityObject;

static void update_planning_scene(
    const std::vector<UnityObject> unity_objects,
    PlanningSceneInterface         planning_scene_interface
) {
    const auto known_ids = planning_scene_interface.getObjects();
    static const std::string PLANNING_FRAME = "arm_base_link";
    std::vector<moveit_msgs::CollisionObject> scene_objects_list;

    // for each object received from Unity
    for (const UnityObject object : unity_objects) {
        // create a corresponding MoveIt Message
        moveit_msgs::CollisionObject scene_object;
        scene_object.id = object.id.data;
        scene_object.header.frame_id = PLANNING_FRAME;
        scene_object.operation = scene_object.ADD;

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        // During the conversion to ROS coordinate space, the y value is
        // inverted z in Unity
        primitive.dimensions[primitive.BOX_X] = object.scale.x;
        // -x in Unity, notice the negative
        primitive.dimensions[primitive.BOX_Y] = -object.scale.y;
        // y in Unity
        primitive.dimensions[primitive.BOX_Z] = object.scale.z;
        scene_object.primitives.push_back(primitive);

        // define the pose
        // PLANNING_FRAME is positioned at (0,0,0) for both side so no
        // transformation needed
        geometry_msgs::Pose primitive_pose;
        primitive_pose.orientation = object.orientation;
        primitive_pose.position = object.position;
        scene_object.primitive_poses.push_back(primitive_pose);

        // Add the message to the list of messages to be sent
        scene_objects_list.push_back(scene_object);
    }

    // Apply the object to planning_scene (Blocking until finished!)
    planning_scene_interface.applyCollisionObjects(scene_objects_list);
}

void unity_targets_subs_handler(const UnityRequest::ConstPtr& message) {
    static const std::string   ARM_PLANNING_GROUP = "robot_arm";
    static const std::string   GRIPPER_PLANNING_GROUP = "robot_gripper";
    static const ros::Duration sleep_timer = ros::Duration(1, 500000);
    ROS_INFO("Received target message from Unity.");

    MoveGroupInterface     arm_move_group_interface(ARM_PLANNING_GROUP);
    MoveGroupInterface     gripper_move_group_interface(GRIPPER_PLANNING_GROUP);
    const PlanningSceneInterface planning_scene_interface;

    // Allow replanning if scene change, would come in useful when scene change?
    arm_move_group_interface.allowReplanning(true);
    // Allow replan attempt in case the planner simply didnt find a path, there
    // are time when it does that
    arm_move_group_interface.setNumPlanningAttempts(5);

    // Same config for gripper
    gripper_move_group_interface.setNumPlanningAttempts(5);

    // Uncomment if you want to check Unity joint control script
    // debug_joint(move_group_interface);

    // Build the planning scene
    update_planning_scene(message->static_objects, planning_scene_interface);

    // Let's start plan and execute!
    // Pre-Grasp pose
    ROS_INFO("Planning and executing pre-grasp pose");
    arm_move_group_interface.setPoseTarget(message->pick_pose, "arm_tcp_link");
    if (arm_move_group_interface.move() !=
        moveit::core::MoveItErrorCode::SUCCESS) {
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

    // [TODO]: Attach the cube here to arm_tcp_link

    // Close the gripper
    gripper_move_group_interface.setNamedTarget("gripper_close");
    gripper_move_group_interface.move();

    // Pickup Pose
    ROS_INFO("Planning and executing pickup pose");
    arm_move_group_interface.setPoseTarget(message->pick_pose, "arm_tcp_link");
    if (arm_move_group_interface.move() !=
        moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("Failed to move to pickup pose, exiting");
        return;
    }
    sleep_timer.sleep();
    ROS_INFO("Pickup pose executed");

    // Pre-place Pose
    ROS_INFO("Planning and executing pre-place pose");
    arm_move_group_interface.setPoseTarget(message->place_pose, "arm_tcp_link");
    if (arm_move_group_interface.move() !=
        moveit::core::MoveItErrorCode::SUCCESS) {
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

    // [TODO]: Detach the cube here from arm_tcp_link

    // Lift-up Pose
    ROS_INFO("Planning and executing lift-up pose");
    arm_move_group_interface.setPoseTarget(message->place_pose, "arm_tcp_link");
    if (arm_move_group_interface.move() !=
        moveit::core::MoveItErrorCode::SUCCESS) {
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