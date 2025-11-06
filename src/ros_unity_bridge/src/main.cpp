#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/spinner.h"
#include "ros/subscriber.h"
#include "ros_unity_messages/SyncUnityObjects.h"
#include "ros_unity_messages/UnityRequest.h"
#include "unity_objects_listener.hpp"
#include "unity_targets_listener.hpp"

int main(int argc, char** argv) {
    ROS_INFO("Starting unity_bridge node");
    ros::init(argc, argv, "unity_bridge");
    ros::NodeHandle node_handle;

    // 2 thread, one thread for subscriber callback, one thread for moveit
    // 2 thread for unity objects, probably
    // callback
    ros::AsyncSpinner spinner = ros::AsyncSpinner(4);

    spinner.start();

    // Unity Targets subscriber
    ROS_INFO("Registering unity_targets subscriber");
    const ros::Subscriber unity_targets_subscriber =
        node_handle.subscribe<ros_unity_messages::UnityRequest>(
            "/unity_bridge/unity_targets", 1000, unity_targets_subs_handler
        );

    // Unity Objects subscriber
    ROS_INFO("Registering unity_objects subscriber");
    const ros::Subscriber unity_objects_subscriber = node_handle.subscribe<ros_unity_messages::SyncUnityObjects>(
        "/unity_bridge/unity_objects", 1000, unity_objects_sub_handler
    );

    ROS_INFO("unity_bridge node ready for action.");

    ros::waitForShutdown();
    return 0;
}