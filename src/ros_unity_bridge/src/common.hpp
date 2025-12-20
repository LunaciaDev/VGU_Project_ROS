#ifndef __UNITY_TARGETS_LISTENER_HPP__
#define __UNITY_TARGETS_LISTENER_HPP__

#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros_unity_messages/UnityDynamicObjects.h"
#include "ros_unity_messages/UnityRequest.h"

// ---

const std::string PLANNING_FRAME = "arm_base_link";

// ---

void bridge_request_handler(
    const ros_unity_messages::UnityRequest::ConstPtr message,
    const ros::Publisher&                            gripper_control_publisher,
    const ros::Publisher&                            dyn_object_sync
);

void dyn_object_handler(
    const ros_unity_messages::UnityDynamicObjects::ConstPtr message,
    const ros::Publisher&                                   send_objects
);

void init_bridge(const ros::NodeHandle& node_handle);

struct EnergyData {
    double move_energy;
    double brake_energy;
    bool   use_estimation;
};

void start_energy_recording(struct EnergyData* data_package);
std::pair<double, double> stop_energy_recording(const struct EnergyData* data_package);

#endif