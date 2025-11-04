#ifndef __JOINT_DEBUG_HPP__
#define __JOINT_DEBUG_HPP__

#include "moveit/move_group_interface/move_group_interface.h"
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

void debug_joint(MoveGroupInterface& move_group_interface);

#endif