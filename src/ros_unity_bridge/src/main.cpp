#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/spinner.h"
#include "ros/subscriber.h"
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
    ros::AsyncSpinner spinner = ros::AsyncSpinner(3);
    spinner.start();

    // Subscriber to unity_targets topic
    // This topic is published by Unity when it want to start the planning and
    // execution of pick and place task.
    ROS_INFO("Registering unity_targets subscriber");
    const ros::Subscriber unity_targets_subscriber =
        node_handle.subscribe<ros_unity_messages::UnityRequest>(
            "/unity_bridge/unity_targets", 0, unity_targets_subs_handler
        );

    ROS_INFO("unity_bridge node ready for action.");

    ros::waitForShutdown();
    return 0;
}