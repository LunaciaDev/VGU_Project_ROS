#ifndef __UNITY_OBJECTS_LISTENER_HPP__
#define __UNITY_OBJECTS_LISTENER_HPP__

#include "ros_unity_messages/SyncUnityObjects.h"

using SyncUnityObjects = ros_unity_messages::SyncUnityObjects;

void unity_objects_sub_handler(const SyncUnityObjects::ConstPtr& message);

#endif