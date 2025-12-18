#include "common.hpp"
#include "moveit_msgs/CollisionObject.h"
#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/spinner.h"
#include "ros/subscriber.h"
#include "ros_unity_messages/GripperControl.h"
#include "ros_unity_messages/UnityDynamicObjects.h"
#include "ros_unity_messages/UnityRequest.h"
#include "std_msgs/Empty.h"

int main(int argc, char** argv) {
    ROS_INFO("Starting unity_bridge node");

    // Initialize the node
    ros::init(argc, argv, "unity_bridge");
    ros::NodeHandle node_handle;
    init_bridge(node_handle);

    // 1 thread run the handler for the subscriber on unity_targets topic.
    // 1 thread to call MoveIt Move Group Interface
    // 1 thread to call MoveIt Planning Scene Interface.
    // 1 thread to publish gripper control message to Unity
    // 1 thread to receive objects from Unity
    ros::AsyncSpinner spinner = ros::AsyncSpinner(5);
    spinner.start();

    ROS_INFO("Starting gripper_control publisher");
    const ros::Publisher gripper_control_publisher =
        node_handle.advertise<ros_unity_messages::GripperControl>(
            "/unity_bridge/gripper_control", 1
        );

    ROS_INFO("Starting dynamic_object sync publisher");
    const ros::Publisher dyn_object_sync =
        node_handle.advertise<std_msgs::Empty>(
            "/unity_bridge/sync_dyn_objects", 1
        );

    const ros::Publisher send_dyn_objects =
        node_handle.advertise<moveit_msgs::CollisionObject>(
            "/collision_object", 1
        );

    // Subscriber to unity_targets topic
    // This topic is published by Unity when it want to start the planning and
    // execution of pick and place task.
    ROS_INFO("Registering unity_targets subscriber");
    const ros::Subscriber unity_targets_subscriber =
        node_handle.subscribe<ros_unity_messages::UnityRequest>(
            "/unity_bridge/unity_targets", 1,
            [gripper_control_publisher, dyn_object_sync](
                const ros_unity_messages::UnityRequest::ConstPtr message
            ) {
                bridge_request_handler(
                    message, gripper_control_publisher, dyn_object_sync
                );
            }
        );

    ROS_INFO("Registering dynamic_object subscriber");
    const ros::Subscriber dyn_object_subscriber =
        node_handle.subscribe<ros_unity_messages::UnityDynamicObjects>(
            "/unity_bridge/dynamic_objects", 1,
            [send_dyn_objects](
                const ros_unity_messages::UnityDynamicObjects::ConstPtr message
            ) { dyn_object_handler(message, send_dyn_objects); }
        );

    ROS_INFO("unity_bridge node ready for action.");

    ros::waitForShutdown();
    return 0;
}