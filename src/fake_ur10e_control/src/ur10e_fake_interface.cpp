/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface that performs a perfect
   control loop for simulation
*/

#include "ur10e_fake_interface.hpp"

UR10eHWInterface::UR10eHWInterface(
    ros::NodeHandle& node_handle,
    urdf::Model*     urdf_model
)
    : ros_control_boilerplate::SimHWInterface(node_handle, urdf_model) {}

void UR10eHWInterface::init() {
    ros_control_boilerplate::SimHWInterface::init();

    joint_position_prev_.resize(num_joints_, 0.0);
    ROS_INFO("UR10e hardware interface ready.");
}

void UR10eHWInterface::read(ros::Duration& elapsed_time) {
    // Do nothing, the state is populated by write.
}

void UR10eHWInterface::write(ros::Duration& elapsed_time) {
    enforceLimits(elapsed_time);

    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id) {
        positionControlSimulation(elapsed_time, joint_id);
    }
}

void UR10eHWInterface::positionControlSimulation(
    ros::Duration&    elapsed_time,
    const std::size_t joint_id
) {
    const double max_delta_pos =
        joint_velocity_limits_[joint_id] * elapsed_time.toSec();

    // Move all the states to the commanded set points at max velocity
    p_error_ = joint_position_command_[joint_id] - joint_position_[joint_id];

    const double delta_pos =
        std::max(std::min(p_error_, max_delta_pos), -max_delta_pos);
    joint_position_[joint_id] += delta_pos;

    // Bypass max velocity p controller:
    // joint_position_[joint_id] = joint_position_command_[joint_id];

    // Calculate velocity based on change in positions
    if (elapsed_time.toSec() > 0) {
        joint_velocity_[joint_id] =
            (joint_position_[joint_id] - joint_position_prev_[joint_id]) /
            elapsed_time.toSec();
    } else
        joint_velocity_[joint_id] = 0;

    // Save last position
    joint_position_prev_[joint_id] = joint_position_[joint_id];
}

void UR10eHWInterface::enforceLimits(ros::Duration& period) {
    // Enforces position and velocity
    pos_jnt_sat_interface_.enforceLimits(period);
}