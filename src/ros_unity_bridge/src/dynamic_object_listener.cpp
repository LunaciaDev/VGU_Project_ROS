#include "common.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "ros/publisher.h"
#include "ros/time.h"

static uint32_t sequence_number = 0;

// ---

void dyn_object_handler(
    const ros_unity_messages::UnityDynamicObjects::ConstPtr message,
    const ros::Publisher&                                   object_publisher
) {
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    for (const auto object : message->dynamic_objs) {
        moveit_msgs::CollisionObject scene_object;
        scene_object.id = object.id.data;
        scene_object.header.frame_id = PLANNING_FRAME;
        scene_object.header.seq = sequence_number++;
        scene_object.header.stamp = ros::Time::now();
        scene_object.operation = scene_object.MOVE;

        // define the pose
        // PLANNING_FRAME is positioned at (0,0,0) for both side so no
        // transformation needed
        scene_object.pose.orientation = object.orientation;
        scene_object.pose.position = object.position;

        object_publisher.publish(scene_object);
    }
}