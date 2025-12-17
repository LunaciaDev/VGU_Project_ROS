#ifndef __UNITY_TARGETS_LISTENER_HPP__
#define __UNITY_TARGETS_LISTENER_HPP__

#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros_unity_messages/UnityRequest.h"

void bridge_request_handler(
    const ros_unity_messages::UnityRequest::ConstPtr& message,
    const ros::Publisher&                             gripper_control_publisher
);

void init_bridge(const ros::NodeHandle& node_handle);

#endif