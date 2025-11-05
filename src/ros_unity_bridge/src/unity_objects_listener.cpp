#include "unity_objects_listener.hpp"

#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/CollisionObject.h"
#include "ros_unity_messages/UnityObject.h"
#include "shape_msgs/SolidPrimitive.h"

using PlanningSceneInterface =
    moveit::planning_interface::PlanningSceneInterface;
using UnityObject = ros_unity_messages::UnityObject;

static const std::string PLANNING_FRAME = "arm_base_link";
static const std::string END_EFFECTOR_LINK = "arm_tcp_link";

void unity_objects_sub_handler(const SyncUnityObjects::ConstPtr& message) {
    /**
     * In theory, this can be published directly from Unity, but..
     * try running rosmsg show moveit_msgs/CollisionObject and you see why
     * we use this interface instead.
     *
     * [TODO]: Experiment with operation type, between ADD MOVE APPEND
     */

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const auto known_ids = planning_scene_interface.getObjects();
    std::vector<moveit_msgs::CollisionObject> scene_objects_list;

    // for each object received from Unity
    for (const UnityObject object : message->objects) {
        // create a corresponding MoveIt Message
        moveit_msgs::CollisionObject scene_object;
        scene_object.id = object.id.data;
        scene_object.header.frame_id = PLANNING_FRAME;
        scene_object.operation = scene_object.MOVE;

        // does the box exist in the planning scene?
        if (known_ids.find(object.id.data) == known_ids.end()) {
            // Does not exists, we add the geometry for the object
            // switch message type to ADD

            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            // During the conversion to ROS coordinate space, the y value is inverted
            // z in Unity
            primitive.dimensions[primitive.BOX_X] = object.scale.x;
            // -x in Unity, notice the negative
            primitive.dimensions[primitive.BOX_Y] = -object.scale.y;
            // y in Unity
            primitive.dimensions[primitive.BOX_Z] = object.scale.z;
            scene_object.operation = scene_object.ADD;
            scene_object.primitives.push_back(primitive);
        }

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

    // Send the messages to planning_scene
    planning_scene_interface.addCollisionObjects(scene_objects_list);

    // Done!
}