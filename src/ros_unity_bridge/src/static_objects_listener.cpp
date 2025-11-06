#include "static_objects_listener.hpp"

#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/CollisionObject.h"
#include "ros_unity_messages/UnityObject.h"
#include "shape_msgs/SolidPrimitive.h"

using PlanningSceneInterface =
    moveit::planning_interface::PlanningSceneInterface;
using UnityObject = ros_unity_messages::UnityObject;

static const std::string PLANNING_FRAME = "arm_base_link";

/**
 * This should be received once only, to build static objects in the Scene
 */
void unity_static_objects_sub_handler(const SyncUnityObjects::ConstPtr& message) {
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const auto known_ids = planning_scene_interface.getObjects();
    std::vector<moveit_msgs::CollisionObject> scene_object_messages;

    moveit_msgs::CollisionObject              scene_objects;
    scene_objects.id = "static_unity_object";
    scene_objects.header.frame_id = PLANNING_FRAME;
    scene_objects.operation = moveit_msgs::CollisionObject::MOVE;

    // for each object received from Unity
    for (const UnityObject object : message->objects) {
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        // z in Unity
        primitive.dimensions[primitive.BOX_X] = object.scale.x;
        // -x in Unity, notice the negative - Scale cannot be negative!
        primitive.dimensions[primitive.BOX_Y] = -object.scale.y;
        // y in Unity
        primitive.dimensions[primitive.BOX_Z] = object.scale.z;
        scene_objects.primitives.push_back(primitive);

        // define the pose
        // PLANNING_FRAME is positioned at (0,0,0) for both side so no
        // transformation needed
        geometry_msgs::Pose primitive_pose;
        primitive_pose.orientation = object.orientation;
        primitive_pose.position = object.position;
        scene_objects.primitive_poses.push_back(primitive_pose);
    }

    scene_object_messages.push_back(scene_objects);

    // Send the messages to planning_scene
    planning_scene_interface.addCollisionObjects(scene_object_messages);

    // Done!
}