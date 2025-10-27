#include <cstdio>
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/service_server.h"
#include "ros_unity_integration/IntegrationService.h"

using IntegrationService = ros_unity_integration::IntegrationService;

bool test(IntegrationService::Request& request, IntegrationService::Response& response)
{
    printf("Hello!");
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_planner");
    ros::NodeHandle node_handle;

    ros::ServiceServer service = node_handle.advertiseService("IntegrationService", test);
    ROS_INFO("Service Ready");
    ros::spin();

    return 0;
}