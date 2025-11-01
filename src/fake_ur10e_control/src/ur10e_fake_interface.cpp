#include "ur10e_fake_interface.hpp"
#include "ros/duration.h"

UR10eHWInterface::UR10eHWInterface(
    ros::NodeHandle& node_handle,
    urdf::Model*     urdf_model
)
    : ros_control_boilerplate::GenericHWInterface(node_handle, urdf_model) {
    ROS_INFO("UR10e hardware interface ready.");
}

void UR10eHWInterface::read(ros::Duration& elapsed_time) {
    // Do nothing, the state is populated by write.
}

void UR10eHWInterface::write(ros::Duration& elapsed_time) {
    enforceLimits(elapsed_time);

    // Snap to the new position
    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id) {
        joint_position_[joint_id] += joint_position_command_[joint_id];
    }
}

void UR10eHWInterface::enforceLimits(ros::Duration& period) {
    // Enforces position and velocity
    pos_jnt_sat_interface_.enforceLimits(period);
}