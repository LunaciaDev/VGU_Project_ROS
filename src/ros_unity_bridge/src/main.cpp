#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/spinner.h"
#include "ros/subscriber.h"
#include "ros_unity_messages/GripperControl.h"
#include "ros_unity_messages/UnityRequest.h"
#include "unity_targets_listener.hpp"

int main(int argc, char** argv) {
    ROS_INFO("Starting unity_bridge node");

    // Initialize the node
    ros::init(argc, argv, "unity_bridge");
    ros::NodeHandle node_handle;

    // Create 3 threads for the node.
    // 1 thread run the handler for the subscriber on unity_targets topic.
    // 1 thread to call MoveIt Move Group Interface
    // 1 thread to call MoveIt Planning Scene Interface.
    // 1 thread to publish gripper control message to Unity
    ros::AsyncSpinner spinner = ros::AsyncSpinner(4);
    spinner.start();

    // Attach padding to the robot gripper
    {
        moveit::planning_interface::MoveGroupInterface move_group_interface(
            "robot_arm"
        );
        moveit::planning_interface::PlanningSceneInterface
            planning_scene_interface;

        // Create our collision box
        shape_msgs::SolidPrimitive gripper_padding;
        gripper_padding.type = gripper_padding.BOX;
        gripper_padding.dimensions = {0.05, 0.04, 0.12};

        geometry_msgs::Pose padding_pose;
        padding_pose.orientation.w = 0;
        padding_pose.orientation.x = 0;
        padding_pose.orientation.y = 0;
        padding_pose.orientation.z = 0;
        padding_pose.position.x = 0;
        padding_pose.position.y = -0.01;
        padding_pose.position.z = 0.01;

        moveit_msgs::CollisionObject left_gripper_padding;
        moveit_msgs::CollisionObject right_gripper_padding;

        left_gripper_padding.id = "left_gripper_padding";
        left_gripper_padding.header.frame_id = "gripper_left_inner_finger";
        left_gripper_padding.operation = left_gripper_padding.ADD;
        left_gripper_padding.primitives = {gripper_padding};
        left_gripper_padding.primitive_poses = {padding_pose};

        right_gripper_padding.id = "right_gripper_padding";
        right_gripper_padding.header.frame_id = "gripper_right_inner_finger";
        right_gripper_padding.operation = right_gripper_padding.ADD;
        right_gripper_padding.primitives = {gripper_padding};
        right_gripper_padding.primitive_poses = {padding_pose};

        ROS_INFO("Adding object to the scene");

        planning_scene_interface.applyCollisionObjects(
            {left_gripper_padding, right_gripper_padding}
        );

        ROS_INFO("Attaching padding to robot");

        move_group_interface.attachObject(
            "left_gripper_padding", "gripper_left_inner_finger",
            {"gripper_left_inner_knuckle", "gripper_left_outer_knuckle",
             "gripper_left_inner_finger"}
        );
        move_group_interface.attachObject(
            "right_gripper_padding", "gripper_right_inner_finger",
            {"gripper_right_inner_knuckle", "gripper_right_outer_knuckle",
             "gripper_right_inner_finger"}
        );
    }

    ROS_INFO("Starting gripper_control publisher");
    const ros::Publisher gripper_control_publisher =
        node_handle.advertise<ros_unity_messages::GripperControl>(
            "/unity_bridge/gripper_control", 0
        );

    // Subscriber to unity_targets topic
    // This topic is published by Unity when it want to start the planning and
    // execution of pick and place task.
    ROS_INFO("Registering unity_targets subscriber");
    const ros::Subscriber unity_targets_subscriber =
        node_handle.subscribe<ros_unity_messages::UnityRequest>(
            "/unity_bridge/unity_targets", 0,
            [gripper_control_publisher](
                const ros_unity_messages::UnityRequest::ConstPtr& message
            ) {
                unity_targets_subs_handler(message, gripper_control_publisher);
            }
        );

    ROS_INFO("unity_bridge node ready for action.");

    ros::waitForShutdown();
    return 0;
}