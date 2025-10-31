#ifndef __MACROS_HPP__
#define __MACROS_HPP__

#include "ros/console.h"

#define INFO(log_string) ROS_INFO_NAMED(LOG_NAME, log_string)
#define DEBUG(log_string) ROS_DEBUG_NAMED(LOG_NAME, log_string)
#define ERROR(log_string) ROS_DEBUG_NAMED(LOG_NAME, log_string)

#endif