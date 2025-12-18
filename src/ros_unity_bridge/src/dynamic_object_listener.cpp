#include "common.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

void dyn_object_handler(
    const ros_unity_messages::UnityDynamicObjects::ConstPtr message
) {
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::CollisionObject>          scene_objects_list;

    for (const auto object : message->dynamic_objs) {
        moveit_msgs::CollisionObject scene_object;
        scene_object.id = object.id.data;
        scene_object.header.frame_id = PLANNING_FRAME;
        scene_object.operation = scene_object.MOVE;

        // define the pose
        // PLANNING_FRAME is positioned at (0,0,0) for both side so no
        // transformation needed
        geometry_msgs::Pose primitive_pose;
        primitive_pose.orientation = object.orientation;
        primitive_pose.position = object.position;
        scene_object.primitive_poses.push_back(primitive_pose);

        // Add the message to the list of messages to be sent
        scene_objects_list.push_back(scene_object);
    }

    planning_scene_interface.applyCollisionObjects(scene_objects_list);
}