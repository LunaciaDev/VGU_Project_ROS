#include "unity_targets_listener.hpp"

#include <cstdio>
#include <cstdlib>
#include <unordered_map>

#include "joint_debug.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/utils/moveit_error_code.h"
#include "moveit/warehouse/moveit_message_storage.h"
#include "moveit/warehouse/planning_scene_storage.h"
#include "moveit_msgs/CollisionObject.h"
#include "ros/console.h"
#include "ros_unity_messages/UnityObject.h"
#include "rviz_visual_tools/rviz_visual_tools.h"
#include "warehouse_ros/database_connection.h"

// ---

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using PlanningSceneInterface =
    moveit::planning_interface::PlanningSceneInterface;
using UnityRequest = ros_unity_messages::UnityRequest;
using UnityObject = ros_unity_messages::UnityObject;
using Plan = moveit::planning_interface::MoveGroupInterface::Plan;
using MoveItStatus = moveit::core::MoveItErrorCode;
using DbConnectionPtr = warehouse_ros::DatabaseConnection::Ptr;
using PlanningSceneStorage = moveit_warehouse::PlanningSceneStorage;

// ---

static const int           PLANNING_ATTEMPTS = 5;
static const double        TIME_PER_ATTEMPT = 10;
static const std::string   PLANNING_FRAME = "arm_base_link";
static const std::string   ARM_PLANNING_GROUP = "robot_arm";
static const std::string   GRIPPER_PLANNING_GROUP = "robot_gripper";

// Planning statistic
static double            planning_time = 0;
static int               total_attempts = 0;
static int               failed_attempts = 0;
static const std::string associated_joint_name[6] = {
    "arm_elbow_joint",   "arm_shoulder_lift_joint", "arm_shoulder_pan_joint",
    "arm_wrist_1_joint", "arm_wrist_2_joint",       "arm_wrist_3_joint"
};
static std::unordered_map<std::string, double> total_joint_trajectory;
static std::unordered_map<std::string, double> previous_joint_position;

// ---

/**
 * Populate the Planning Scene with objects from Unity.
 * This function is blocking until MoveIt confirms that all object has been
 * added into the Scene.
 */
static void update_planning_scene(
    const std::vector<UnityObject>& unity_objects,
    PlanningSceneInterface&         planning_scene_interface
) {
    const auto known_ids = planning_scene_interface.getObjects();
    std::vector<moveit_msgs::CollisionObject> scene_objects_list;

    // for each object received from Unity
    for (const UnityObject object : unity_objects) {
        // create a corresponding MoveIt Message
        moveit_msgs::CollisionObject scene_object;
        scene_object.id = object.id.data;
        scene_object.header.frame_id = PLANNING_FRAME;
        scene_object.operation = scene_object.ADD;

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);

        // During the conversion to ROS coordinate space, the y value is
        // inverted.

        // z in Unity
        primitive.dimensions[primitive.BOX_X] = object.scale.x;
        // -x in Unity, notice the negative
        primitive.dimensions[primitive.BOX_Y] = -object.scale.y;
        // y in Unity
        primitive.dimensions[primitive.BOX_Z] = object.scale.z;
        scene_object.primitives.push_back(primitive);

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

    // Apply the object to planning_scene (Blocking until finished!)
    planning_scene_interface.applyCollisionObjects(scene_objects_list);
}

/**
 * Draw an arrow at each location sent from Unity on RViz.
 */
static void generate_markers(const UnityRequest::ConstPtr& message) {
    auto visual_tool = new rviz_visual_tools::RvizVisualTools(
        "arm_base_link", "/rviz_visual_markers"
    );
    visual_tool->deleteAllMarkers();

    // For each pose, create an arrow for it.
    visual_tool->publishArrow(
        message->pre_pick_location, rviz_visual_tools::RED
    );
    visual_tool->publishArrow(message->pick_location, rviz_visual_tools::GREEN);
    visual_tool->publishArrow(
        message->pre_place_location, rviz_visual_tools::BLUE
    );
    visual_tool->publishArrow(
        message->place_location, rviz_visual_tools::YELLOW
    );
    visual_tool->trigger();
}

/**
 * Generate the target cube.
 * This is a workaround for an issue where the target cube cannot exist within
 * the scene as for the gripper to grip the cube, its finger joint MUST collide
 * with the cube, which is not allowed by MoveIt.
 * This work around should be removed once we can handle dynamic objects.
 * [FIXME]: Remove this after dynamic objects is implemented.
 * [NOTE]: Once it is implemented, we could attach the cube into the arm
 * specifying that collision between finger and cube is fine? Then the gripper
 * can lock into that. Also experiment if that attachment would conflict with
 * dynamic object code.
 */
static moveit_msgs::CollisionObject generate_cube(
    const geometry_msgs::Pose& location
) {
    // The CUBE is located at pick_pose, size 5cm
    moveit_msgs::CollisionObject scene_object;
    scene_object.id = "CUBE";
    scene_object.header.frame_id = PLANNING_FRAME;
    scene_object.operation = scene_object.ADD;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3, 0.05);
    scene_object.primitives.push_back(primitive);

    scene_object.primitive_poses.push_back(location);

    return scene_object;
}

/**
 * Adapter for planning and executing a trajectory, with profiling.
 *
 * Return 0 on sucessfully executing the trajectory, -1 otherwise.
 */
static int planning_with_profiling(
    MoveGroupInterface& arm_move_group_interface
) {
    Plan plan = Plan();
    int  attempt = 0;

    while (attempt < PLANNING_ATTEMPTS) {
        auto status = arm_move_group_interface.plan(plan);

        if (status == MoveItStatus::SUCCESS) {
            break;
        }

        total_attempts += 1;
        failed_attempts += 1;
        attempt += 1;

        if (attempt >= PLANNING_ATTEMPTS) {
            return -1;
        }
    }

    total_attempts += 1;
    planning_time += plan.planning_time_;
    const std::vector<std::string> joint_names =
        plan.trajectory_.joint_trajectory.joint_names;

    for (const auto waypoints : plan.trajectory_.joint_trajectory.points) {
        for (int i = 0; i < 6; i++) {
            const auto joint_name = joint_names[i];
            total_joint_trajectory[joint_name] +=
                abs(waypoints.positions[i] -
                    previous_joint_position[joint_name]);
            previous_joint_position[joint_name] = waypoints.positions[i];
        }
    }

    // execute the plan
    if (arm_move_group_interface.execute(plan) != MoveItStatus::SUCCESS) {
        return -1;
    }

    return 0;
}

/**
 * Adapter for executing a trajectory, without profiling.
 *
 * Return 0 on success, and -1 otherwise.
 */
static int planning_no_profiling(MoveGroupInterface& arm_move_group_interface) {
    if (arm_move_group_interface.move() != MoveItStatus::SUCCESS) {
        return -1;
    }
    return 0;
}

/**
 * Write the result into log. Back up in case for some reason we cannot open the
 * csv.
 */
static void write_log_result() {
    ROS_INFO("Total planning time: %.5f", planning_time);
    for (const auto joint_moved_value : total_joint_trajectory) {
        ROS_INFO(
            "%s expected to move %.5f radians", joint_moved_value.first.c_str(),
            joint_moved_value.second
        );
    }
    ROS_INFO("Failed planning attempts: %d", failed_attempts);
    ROS_INFO("Total attempts: %d", total_attempts);
}

/**
 * Write the result of movement to a csv.
 *
 * If the csv cannot be opened, echo result into the console.
 */
static void write_result() {
    // C-style since that's what I am familiar with
    FILE* is_handle_exist = fopen("result.csv", "r");
    FILE* result_file_handle;

    // we did not create the file.
    if (is_handle_exist == NULL) {
        result_file_handle = fopen("result.csv", "a");

        if (result_file_handle == NULL) {
            // write to log
            write_log_result();
            return;
        }

        fprintf(
            result_file_handle,
            "planning_time,%s,%s,%s,%s,%s,%s,failed_attempts,total_attempts\n",
            associated_joint_name[0].c_str(), associated_joint_name[1].c_str(),
            associated_joint_name[2].c_str(), associated_joint_name[3].c_str(),
            associated_joint_name[4].c_str(), associated_joint_name[5].c_str()
        );
    } else {
        result_file_handle = fopen("result.csv", "a");

        if (result_file_handle == NULL) {
            // write to log
            write_log_result();
            return;
        }
    }

    // Write the result
    fprintf(result_file_handle, "%.6f,", planning_time);

    for (int i = 0; i < 6; i++) {
        fprintf(
            result_file_handle, "%.6f,",
            total_joint_trajectory[associated_joint_name[i]]
        );
    }

    fprintf(result_file_handle, "%d,%d\n", failed_attempts, total_attempts);

    // Flush the result into file
    fflush(result_file_handle);
}

/**
 * Handler for request to execute pick and place from Unity.
 *
 * We first build the static objects in the planning scene according to the
 * message, then execute pick and place.
 *
 * The pick and place consist of these step:
 * - Move to top of the target.
 * - Lower the gripper.
 * - Grasp the cube.
 * - Pull the gripper up.
 * - Move to top of where the target needed to be placed.
 * - Lower the gripper.
 * - Release the cube.
 * - Pull the gripper up.
 * - Return the robot to all-zero position.
 */
void unity_targets_subs_handler(const UnityRequest::ConstPtr& message) {
    ROS_INFO("Received planning request from Unity.");

    const DbConnectionPtr DB_CONNECTION = moveit_warehouse::loadDatabase();

    // [FIXME]: Take the parameter from rosparam instead of hardcoded.
    DB_CONNECTION->setParams("localhost", 1234);
    if (!DB_CONNECTION->connect()) {
        ROS_ERROR("Cannot connect to warehouse");
        return;
    }

    MoveGroupInterface     arm_move_group_interface(ARM_PLANNING_GROUP);
    MoveGroupInterface     gripper_move_group_interface(GRIPPER_PLANNING_GROUP);
    PlanningSceneInterface planning_scene_interface;
    PlanningSceneStorage   planning_scene_storage(DB_CONNECTION);

    const moveit_msgs::CollisionObject the_cube =
        generate_cube(message->pick_location);

    // [DEBUG]: Show the location of all poses on rviz
    // generate_markers(message);

    // Allow replanning if scene change, would come in useful in dynamic object
    // scenario?
    arm_move_group_interface.allowReplanning(true);
    // Allow replan attempt in case the planner simply didnt find a path, there
    // are time when it does that
    arm_move_group_interface.setNumPlanningAttempts(PLANNING_ATTEMPTS);
    // Maximum 10s per attempt
    arm_move_group_interface.setPlanningTime(TIME_PER_ATTEMPT);

    // Same config for gripper
    gripper_move_group_interface.setNumPlanningAttempts(PLANNING_ATTEMPTS);
    // Set gripper to always use OMPL
    gripper_move_group_interface.setPlanningPipelineId("ompl");

    // [DEBUG]: check Unity joint control script
    // debug_joint(move_group_interface);

    // Set execution mode (With/Without profiling)
    // [TODO]: Expose this as an option
    auto planning_call = planning_with_profiling;
    // Initialize maps
    for (const std::string joint_name : associated_joint_name) {
        total_joint_trajectory[joint_name] = 0;
        previous_joint_position[joint_name] = 0;
    }
    // auto planning_call = planning_pure;

    // Build the planning scene
    update_planning_scene(message->static_objects, planning_scene_interface);
    ROS_INFO("Planning Scene updated with static objects.");

    // Let's start plan and execute!
    // Pre-Grasp pose
    ROS_INFO("Planning and executing pre-grasp pose");
    arm_move_group_interface.setPoseTarget(
        message->pre_pick_location, "arm_tcp_link"
    );
    if (planning_call(arm_move_group_interface) != 0) {
        ROS_ERROR("Failed to move to pre_grasp pose, exiting");
        return;
    }
    ROS_INFO("Pre-grasp pose executed");

    // Open the gripper
    gripper_move_group_interface.setNamedTarget("gripper_open");
    gripper_move_group_interface.move();
    ROS_INFO("Gripper opened");

    // Pick pose
    ROS_INFO("Planning and executing pick pose");
    arm_move_group_interface.setPoseTarget(
        message->pick_location, "arm_tcp_link"
    );
    if (planning_call(arm_move_group_interface) != 0) {
        ROS_ERROR("Failed to move to pick pose, exiting");
        return;
    }
    ROS_INFO("Pick pose executed");

    // Close the gripper
    gripper_move_group_interface.setNamedTarget("gripper_close");
    gripper_move_group_interface.move();
    ROS_INFO("Gripper closed");

    // Add cube to PlanningScene
    planning_scene_interface.applyCollisionObject(the_cube);
    // Attach to arm_tcp_link, specifying safe self-collision with gripper
    // fingers
    arm_move_group_interface.attachObject(
        "CUBE", "arm_tcp_link",
        {"gripper_right_inner_finger", "gripper_left_inner_finger"}
    );

    // Pickup Pose
    ROS_INFO("Planning and executing pickup pose");
    arm_move_group_interface.setPoseTarget(
        message->pre_pick_location, "arm_tcp_link"
    );
    if (planning_call(arm_move_group_interface) != 0) {
        ROS_ERROR("Failed to move to pickup pose, exiting");
        return;
    }
    ROS_INFO("Pickup pose executed");

    // Pre-place Pose
    ROS_INFO("Planning and executing pre-place pose");
    arm_move_group_interface.setPoseTarget(
        message->pre_place_location, "arm_tcp_link"
    );
    if (planning_call(arm_move_group_interface) != 0) {
        ROS_ERROR("Failed to move to pre_place pose, exiting");
        return;
    }
    ROS_INFO("Pre-place pose executed");

    // Place Pose
    ROS_INFO("Planning and executing place pose");
    arm_move_group_interface.setPoseTarget(
        message->place_location, "arm_tcp_link"
    );
    if (planning_call(arm_move_group_interface) != 0) {
        ROS_ERROR("Failed to move to place pose, exiting");
        return;
    }
    ROS_INFO("Place pose executed");

    // Open the gripper
    gripper_move_group_interface.setNamedTarget("gripper_open");
    gripper_move_group_interface.move();
    ROS_INFO("Gripper opened");

    // Detach the cube from the arm, and remove the cube from the scene.
    arm_move_group_interface.detachObject("CUBE");
    planning_scene_interface.removeCollisionObjects({"CUBE"});

    // Lift-up Pose
    ROS_INFO("Planning and executing lift-up pose");
    arm_move_group_interface.setPoseTarget(
        message->pre_place_location, "arm_tcp_link"
    );
    if (planning_call(arm_move_group_interface) != 0) {
        ROS_ERROR("Failed to move to lift-up pose, exiting");
        return;
    }
    ROS_INFO("Lift-up pose executed");

    // Return gripper to neutral
    gripper_move_group_interface.setNamedTarget("gripper_neutral");
    gripper_move_group_interface.move();
    ROS_INFO("Gripper returned to neutral state.");

    // Return to starting position
    {
        std::vector<double> joint_group_position;
        joint_group_position.resize(6, 0);
        arm_move_group_interface.setJointValueTarget(joint_group_position);
        if (planning_call(arm_move_group_interface) != 0) {
            ROS_ERROR("Failed to move to all-zero pose, exiting");
            return;
        }
    }
    ROS_INFO("All-zero pose executed");
    ROS_INFO("Pick and Place task finished.");

    planning_scene_interface.clear();

    // Write result to file
    write_result();
}