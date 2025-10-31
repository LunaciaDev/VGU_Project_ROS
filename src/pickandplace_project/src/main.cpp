#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/spinner.h"
#include "ros/subscriber.h"

#include "macros.hpp"
#include "unity_targets_listener.hpp"

static const char* LOG_NAME = "unity_bridge";

int main(int argc, char** argv) {
    INFO("Starting unity_bridge node");
    ros::init(argc, argv, "unity_bridge");
    ros::NodeHandle node_handle;

    // 2 thread, one thread for subscriber callback, one thread for moveit
    // callback
    ros::AsyncSpinner spinner = ros::AsyncSpinner(2);

    spinner.start();

    // Unity Targets subscriber
    INFO("Registering unity_targets subscriber");
    const ros::Subscriber subscriber =
        node_handle.subscribe("unity_targets", 1000, unity_targets_subs_handler);

    // Unity Objects subscriber
    // [TODO]

    // Move Target Action Goal Subscriber
    // [TODO]

    // Plan Publisher to Unity
    // [TODO]

    INFO("unity_bridge node ready for action.");

    ros::waitForShutdown();
    return 0;
}