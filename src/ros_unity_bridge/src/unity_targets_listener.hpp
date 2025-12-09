#ifndef __UNITY_TARGETS_LISTENER_HPP__
#define __UNITY_TARGETS_LISTENER_HPP__

#include "ros/publisher.h"
#include "ros_unity_messages/UnityRequest.h"

void unity_targets_subs_handler(
    const ros_unity_messages::UnityRequest::ConstPtr& message,
    const ros::Publisher gripper_control_publisher
);

#endif