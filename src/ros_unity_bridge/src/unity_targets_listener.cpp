#include "unity_targets_listener.hpp"

#include <boost/smart_ptr/shared_ptr.hpp>
#include <unordered_map>

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/utils/moveit_error_code.h"
#include "moveit/warehouse/moveit_message_storage.h"
#include "moveit/warehouse/planning_scene_storage.h"
#include "moveit_msgs/CollisionObject.h"
#include "ros/console.h"
#include "ros/duration.h"
#include "ros/node_handle.h"
#include "ros/topic.h"
#include "ros_unity_messages/GripperControl.h"
#include "ros_unity_messages/UnityObject.h"
#include "rtde_echo/RtdeData.h"
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
using MotionPlanRequest = moveit_msgs::MotionPlanRequest;
using GripperControl = ros_unity_messages::GripperControl;

// ---

static const std::string   PLANNING_FRAME = "arm_base_link";
static const ros::Duration GRIPPER_CONTROL_DELAY = ros::Duration(0, 500000);
enum PathSection { PrePick = 0, PrePlace = 1, Home = 2, Untracked };
struct DatabaseInfo {
    std::string hostname;
    int         port;
};

// ---

static int         parallel_planners;
static double      plan_timeout;
static double      replan_delay;
static int         replan_attempt;
static std::string main_pipeline;
static std::string side_pipeline;
static std::string main_planner;
static std::string side_planner;
static int (*planner_adapter)(
    MoveGroupInterface&,
    PlanningSceneStorage&,
    const std::string&,
    PathSection
);
static bool                generate_marker;
static struct DatabaseInfo db_info;

// Planning statistic
static double            planning_time[3] = {0, 0, 0};
static int               total_attempts[3] = {0, 0, 0};
static int               failed_attempts[3] = {0, 0, 0};
static double            energy_consumed[3] = {0, 0, 0};
static double            braking_energy[3] = {0, 0, 0};
static const std::string associated_joint_name[6] = {
    "arm_elbow_joint",   "arm_shoulder_lift_joint", "arm_shoulder_pan_joint",
    "arm_wrist_1_joint", "arm_wrist_2_joint",       "arm_wrist_3_joint"
};
static const char* output_names[3] = {
    "pre_pick.csv", "pre_place.csv", "home.csv"
};
static std::unordered_map<std::string, double> total_joint_trajectory[3];
static std::unordered_map<std::string, double> previous_joint_position[3];

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

    // Gripper padding
    shape_msgs::SolidPrimitive gripper_padding;
    gripper_padding.type = gripper_padding.BOX;
    gripper_padding.dimensions = {0.05, 0.04, 0.12};

    geometry_msgs::Pose padding_pose;
    padding_pose.orientation.w = 1;
    padding_pose.orientation.x = 0;
    padding_pose.orientation.y = 0;
    padding_pose.orientation.z = 0;
    padding_pose.position.x = 0;
    padding_pose.position.y = -0.01;
    padding_pose.position.z = 0.01;

    moveit_msgs::CollisionObject left_gripper_padding;
    moveit_msgs::CollisionObject right_gripper_padding;

    left_gripper_padding.id = "left_gripper_padding";
    left_gripper_padding.header.frame_id = "gripper_left_inner_finger";
    left_gripper_padding.operation = left_gripper_padding.ADD;
    left_gripper_padding.primitives = {gripper_padding};
    left_gripper_padding.primitive_poses = {padding_pose};

    right_gripper_padding.id = "right_gripper_padding";
    right_gripper_padding.header.frame_id = "gripper_right_inner_finger";
    right_gripper_padding.operation = right_gripper_padding.ADD;
    right_gripper_padding.primitives = {gripper_padding};
    right_gripper_padding.primitive_poses = {padding_pose};

    scene_objects_list.push_back(left_gripper_padding);
    scene_objects_list.push_back(right_gripper_padding);

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

static int planning_no_cache(
    MoveGroupInterface&   move_group_interface,
    PlanningSceneStorage& planning_scene_storage,
    const std::string&    scene_name,
    PathSection           path_section
) {
    Plan plan = Plan();

    while (move_group_interface.plan(plan) != MoveItStatus::SUCCESS) {
        continue;
    }

    if (move_group_interface.execute(plan) != MoveItStatus::SUCCESS) {
        return -1;
    }

    return 0;
}

/**
 * Adapter for planning and executing a trajectory, with profiling.
 * This function will NOT write, nor use, motion planning cache.
 *
 * Return 0 on sucessfully executing the trajectory, -1 otherwise.
 */
static int planning_with_profiling(
    MoveGroupInterface&   move_group_interface,
    PlanningSceneStorage& planning_scene_storage,
    const std::string&    scene_name,
    PathSection           section
) {
    ROS_INFO("Using a profiled motion planning adapter");
    Plan plan = Plan();

    // do not profile untracked section
    if (section == PathSection::Untracked) {
        return planning_no_cache(
            move_group_interface, planning_scene_storage, scene_name, section
        );
    }

    while (move_group_interface.plan(plan) != MoveItStatus::SUCCESS) {
        total_attempts[section] += 1;
        failed_attempts[section] += 1;
    }

    total_attempts[section] += 1;
    planning_time[section] += plan.planning_time_;
    const std::vector<std::string> joint_names =
        plan.trajectory_.joint_trajectory.joint_names;

    for (const auto waypoints : plan.trajectory_.joint_trajectory.points) {
        for (int i = 0; i < 6; i++) {
            const auto joint_name = joint_names[i];
            total_joint_trajectory[section][joint_name] +=
                abs(waypoints.positions[i] -
                    previous_joint_position[section][joint_name]);
            previous_joint_position[section][joint_name] =
                waypoints.positions[i];
        }
    }

    // Grab current power data
    boost::shared_ptr<const rtde_echo::RtdeData> data =
        ros::topic::waitForMessage<rtde_echo::RtdeData>(
            "/unity_bridge/rtde_data"
        );

    double before_energy, before_brake;

    if (data) {
        before_brake = data->braking_energy_dissipated;
        before_energy = data->energy_consumed;
    } else {
        ROS_ERROR(
            "rtde_echo node abnormality or this node is shutting down, cannot "
            "collect data"
        );
    }

    // execute the plan
    if (move_group_interface.execute(plan) != MoveItStatus::SUCCESS) {
        return -1;
    }

    data = ros::topic::waitForMessage<rtde_echo::RtdeData>(
        "/unity_bridge/rtde_data"
    );

    if (data) {
        energy_consumed[section] = data->energy_consumed - before_energy;
        braking_energy[section] =
            data->braking_energy_dissipated - before_brake;
    } else {
        ROS_ERROR(
            "rtde_echo node abnormality or this node is shutting down, cannot "
            "collect data"
        );
    }

    return 0;
}

/**
 * Adapter for executing a trajectory, without profiling.
 *
 * Return 0 on success, and -1 otherwise.
 */
static int planning_no_profiling(
    MoveGroupInterface&   move_group_interface,
    PlanningSceneStorage& planning_scene_storage,
    const std::string&    scene_name,
    PathSection           section
) {
    ROS_INFO("Using cache-based planning adapter");

    // Do not try the cache for untracked sections
    if (section == PathSection::Untracked) {
        return planning_no_cache(
            move_group_interface, planning_scene_storage, scene_name, section
        );
    }

    std::vector<moveit_warehouse::RobotTrajectoryWithMetadata> plan_results;

    // Do we have any motion plan saved for this request, on this scene?
    switch (section) {
        case PrePick:
            planning_scene_storage.getPlanningResults(
                plan_results, scene_name, "PrePick"
            );
            break;
        case PrePlace:
            planning_scene_storage.getPlanningResults(
                plan_results, scene_name, "PrePlace"
            );
            break;
        case Home:
            planning_scene_storage.getPlanningResults(
                plan_results, scene_name, "Home"
            );
            break;
        case Untracked:
            break;
    }

    // No plan found.
    if (plan_results.empty()) {
        // Create a plan
        Plan              plan = Plan();
        MotionPlanRequest plan_request = MotionPlanRequest();
        move_group_interface.constructMotionPlanRequest(plan_request);

        ROS_INFO("Cannot find a cached motion. Creating a new motion plan.");

        while (move_group_interface.plan(plan) != MoveItStatus::SUCCESS) {
            continue;
        }

        // Execute the plan
        if (move_group_interface.execute(plan) != MoveItStatus::SUCCESS) {
            return -1;
        }

        // Cache the plan
        ROS_INFO("Motion plan execution successful. Caching the plan.");

        // Add the query into the db
        switch (section) {
            case PrePick:
                planning_scene_storage.addPlanningQuery(
                    plan_request, scene_name, "PrePick"
                );
                break;
            case PrePlace:
                planning_scene_storage.addPlanningQuery(
                    plan_request, scene_name, "PrePlace"
                );
                break;
            case Home:
                planning_scene_storage.addPlanningQuery(
                    plan_request, scene_name, "Home"
                );
                break;
            case Untracked:
                // Technically this should be a throw?
                break;
        }

        planning_scene_storage.addPlanningResult(
            plan_request, plan.trajectory_, scene_name
        );

        return 0;
    } else {
        ROS_INFO("Found a cached motion plan. Using it instead of planning.");
        // Use the plan.
        // [FIXME]: The plan may fail due to scene changes. Figure out a way to
        // update the plan. The failure may happen after the robot moved away
        // from the starting position is the problem.
        const moveit_msgs::RobotTrajectory cached_trajectory =
            *plan_results[0].get();

        if (move_group_interface.execute(cached_trajectory) !=
            MoveItStatus::SUCCESS) {
            return -1;
        }

        return 0;
    }
}

/**
 * Write the result into log. Back up in case for some reason we cannot open the
 * csv.
 */
static void write_log_result(int index) {
    ROS_INFO("Section: %s", output_names[index]);
    ROS_INFO("Total planning time: %.6f", planning_time[index]);
    for (const auto joint_moved_value : total_joint_trajectory[index]) {
        ROS_INFO(
            "%s expected to move %.6f radians", joint_moved_value.first.c_str(),
            joint_moved_value.second
        );
    }
    ROS_INFO("Failed planning attempts: %d", failed_attempts[index]);
    ROS_INFO("Total attempts: %d", total_attempts[index]);
    ROS_INFO("Power consumed: %.6f", energy_consumed[index]);
    ROS_INFO("Braking power dissipated: %.6f", braking_energy[index]);
}

/**
 * Write the result of movement to a csv.
 *
 * If the csv cannot be opened, echo result into the console.
 */
static void write_result(void) {
    FILE* file_handle;
    int   index = -1;
    // C-style since that's what I am familiar with
    for (const char* filename : output_names) {
        index++;
        file_handle = fopen(filename, "r");

        // The file was not initialized
        if (file_handle == NULL) {
            file_handle = fopen(filename, "w");

            // cannot open this as write for some reason...
            if (file_handle == NULL) {
                write_log_result(index);
                continue;
            }

            // Write the header
            fprintf(
                file_handle,
                "planning_time,%s,%s,%s,%s,%s,%s,failed_attempts,total_"
                "attempts,power_consumed,braking_power_dissipated\n",
                associated_joint_name[0].c_str(),
                associated_joint_name[1].c_str(),
                associated_joint_name[2].c_str(),
                associated_joint_name[3].c_str(),
                associated_joint_name[4].c_str(),
                associated_joint_name[5].c_str()
            );
        } else {
            fclose(file_handle);
            file_handle = fopen(filename, "a");

            if (file_handle == NULL) {
                write_log_result(index);
                continue;
            }
        }

        // Write results
        // planning time
        fprintf(file_handle, "%.6f,", planning_time[index]);
        // joint movement
        for (int joint_index = 0; joint_index < 6; joint_index++) {
            fprintf(
                file_handle, "%.6f,",
                total_joint_trajectory[index]
                                      [associated_joint_name[joint_index]]
            );
        }
        // attempts
        fprintf(
            file_handle, "%d,%d,", failed_attempts[index], total_attempts[index]
        );
        // power
        fprintf(
            file_handle, "%.6f,%.6f\n", energy_consumed[index],
            braking_energy[index]
        );
        // flush and close stream
        fclose(file_handle);
    }
}

void init_bridge(const ros::NodeHandle& node_handle) {
    node_handle.param("/unity_bridge/parallel_planners", parallel_planners, 4);
    node_handle.param("/unity_bridge/planning_timeout", plan_timeout, 30.0);
    node_handle.param("/unity_bridge/replan_attempt", replan_attempt, 5);
    node_handle.param("/unity_bridge/replan_delay", replan_delay, 1.0);

    node_handle.param(
        "/unity_bridge/main_planner_pipeline", main_pipeline, std::string("ompl")
    );
    if (main_pipeline == "ompl") {
        node_handle.param(
            "/unity_bridge/main_planner_id", main_planner, std::string("RRTConnect")
        );
    }
    node_handle.param(
        "/unity_bridge/side_planner_pipeline", side_pipeline, std::string("ompl")
    );
    if (side_pipeline == "ompl") {
        node_handle.param(
            "/unity_bridge/side_planner_id", side_planner, std::string("RRTConnect")
        );
    }

    bool use_cached_path, profiling;
    node_handle.param("/unity_bridge/use_cached_path", use_cached_path, false);
    node_handle.param("/unity_bridge/profiling", profiling, false);

    // Profiling take highest precedence
    if (profiling) {
        planner_adapter = planning_with_profiling;
    }
    // Then caching
    else if (use_cached_path) {
        planner_adapter = planning_no_profiling;
    } else {
        planner_adapter = planning_no_cache;
    }

    node_handle.param("/unity_bridge/generate_pick_place_marker", generate_marker, false);

    node_handle.param("/warehouse_port", db_info.port, 1234);
    node_handle.param(
        "/warehouse_host", db_info.hostname, std::string("localhost")
    );
}

static void switch_main_planner(MoveGroupInterface& move_group_interface) {
    move_group_interface.setPlanningPipelineId(main_pipeline);
    if (!main_planner.empty()) {
        move_group_interface.setPlannerId(main_planner);
    } else {
        move_group_interface.setPlannerId(
            move_group_interface.getDefaultPlannerId()
        );
    }
}

static void switch_side_planner(MoveGroupInterface& move_group_interface) {
    move_group_interface.setPlanningPipelineId(side_pipeline);
    if (!side_pipeline.empty()) {
        move_group_interface.setPlannerId(side_planner);
    } else {
        move_group_interface.setPlannerId(
            move_group_interface.getDefaultPlannerId()
        );
    }
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
void bridge_request_handler(
    const UnityRequest::ConstPtr& message,
    const ros::Publisher&         gripper_control_publisher
) {
    ROS_INFO("Received planning request from Unity.");

    const DbConnectionPtr DB_CONNECTION = moveit_warehouse::loadDatabase();

    DB_CONNECTION->setParams(db_info.hostname, db_info.port);
    if (!DB_CONNECTION->connect()) {
        ROS_ERROR("Cannot connect to warehouse");
        return;
    }

    // Gripper messages
    GripperControl gripper_open = GripperControl();
    gripper_open.gripper_angle = -0.20f;
    GripperControl gripper_close = GripperControl();
    gripper_close.gripper_angle = 0.32f;
    GripperControl gripper_neutral = GripperControl();
    gripper_neutral.gripper_angle = 0.0f;

    MoveGroupInterface                 move_group_interface("robot_arm");
    PlanningSceneInterface             planning_scene_interface;
    PlanningSceneStorage               planning_scene_storage(DB_CONNECTION);

    const moveit_msgs::CollisionObject the_cube =
        generate_cube(message->pick_location);

    if (generate_marker) {
        generate_markers(message);
    }

    // Allow replanning if scene change, would come in useful in dynamic object
    // scenario?
    move_group_interface.allowReplanning(true);
    // How many replan attempt can the robot try?
    move_group_interface.setReplanAttempts(replan_attempt);
    move_group_interface.setReplanDelay(replan_delay);

    // How many planner instance can we run in parallel?
    move_group_interface.setNumPlanningAttempts(parallel_planners);
    // Time waiting before we timeout the planners?
    move_group_interface.setPlanningTime(plan_timeout);

    // Reset stat counters
    for (int index = 0; index < 3; index++) {
        // Joint movement
        for (const std::string joint_name : associated_joint_name) {
            total_joint_trajectory[index][joint_name] = 0;
            previous_joint_position[index][joint_name] = 0;
        }

        // Time
        planning_time[index] = 0;

        // Attempts
        total_attempts[index] = 0;
        failed_attempts[index] = 0;

        // Energy
        energy_consumed[index] = 0;
        braking_energy[index] = 0;
    }

    // Build the planning scene
    update_planning_scene(message->static_objects, planning_scene_interface);
    ROS_INFO("Planning Scene updated with static objects.");

    // Attach gripper padding
    move_group_interface.attachObject(
        "left_gripper_padding", "gripper_left_inner_finger",
        {"gripper_left_inner_knuckle", "gripper_left_outer_knuckle",
         "gripper_left_inner_finger"}
    );
    move_group_interface.attachObject(
        "right_gripper_padding", "gripper_right_inner_finger",
        {"gripper_right_inner_knuckle", "gripper_right_outer_knuckle",
         "gripper_right_inner_finger"}
    );

    // Let's start plan and execute!
    // Pre-Grasp pose
    ROS_INFO("Planning and executing pre-pick pose");
    switch_main_planner(move_group_interface);
    move_group_interface.setPoseTarget(
        message->pre_pick_location, "arm_tcp_link"
    );
    if (planner_adapter(
            move_group_interface, planning_scene_storage,
            message->scene_name.data, PathSection::PrePick
        ) != 0) {
        ROS_ERROR("Failed to move to pre_grasp pose, exiting");
        return;
    }
    ROS_INFO("Pre-grasp pose executed");

    // Open gripper
    GRIPPER_CONTROL_DELAY.sleep();
    gripper_control_publisher.publish(gripper_open);
    ROS_INFO("Gripper opened");
    GRIPPER_CONTROL_DELAY.sleep();

    // Pick pose
    ROS_INFO("Planning and executing pick pose");
    switch_side_planner(move_group_interface);
    move_group_interface.setPoseTarget(message->pick_location, "arm_tcp_link");
    if (planner_adapter(
            move_group_interface, planning_scene_storage,
            message->scene_name.data, PathSection::Untracked
        ) != 0) {
        ROS_ERROR("Failed to move to pick pose, exiting");
        return;
    }
    ROS_INFO("Pick pose executed");

    // Close the gripper
    GRIPPER_CONTROL_DELAY.sleep();
    gripper_control_publisher.publish(gripper_close);
    ROS_INFO("Gripper closed");
    GRIPPER_CONTROL_DELAY.sleep();

    // Add cube to PlanningScene
    planning_scene_interface.applyCollisionObject(the_cube);
    // Attach to arm_tcp_link, specifying safe self-collision with gripper
    // fingers
    move_group_interface.attachObject(
        "CUBE", "arm_tcp_link",
        {"gripper_right_inner_finger", "gripper_left_inner_finger"}
    );

    // Pickup Pose
    ROS_INFO("Planning and executing pickup pose");
    switch_side_planner(move_group_interface);
    move_group_interface.setPoseTarget(
        message->pre_pick_location, "arm_tcp_link"
    );
    if (planner_adapter(
            move_group_interface, planning_scene_storage,
            message->scene_name.data, PathSection::Untracked
        ) != 0) {
        ROS_ERROR("Failed to move to pickup pose, exiting");
        return;
    }
    ROS_INFO("Pickup pose executed");

    // Pre-place Pose
    ROS_INFO("Planning and executing pre-place pose");
    switch_main_planner(move_group_interface);
    move_group_interface.setPoseTarget(message->pre_place_location, "arm_tcp_link");
    if (planner_adapter(
            move_group_interface, planning_scene_storage,
            message->scene_name.data, PathSection::PrePlace
        ) != 0) {
        ROS_ERROR("Failed to move to pre_place pose, exiting");
        return;
    }
    ROS_INFO("Pre-place pose executed");

    // Place Pose
    ROS_INFO("Planning and executing place pose");
    switch_side_planner(move_group_interface);
    move_group_interface.setPoseTarget(message->place_location, "arm_tcp_link");
    if (planner_adapter(
            move_group_interface, planning_scene_storage,
            message->scene_name.data, PathSection::Untracked
        ) != 0) {
        ROS_ERROR("Failed to move to place pose, exiting");
        return;
    }
    ROS_INFO("Place pose executed");

    // Open the gripper
    GRIPPER_CONTROL_DELAY.sleep();
    gripper_control_publisher.publish(gripper_open);
    ROS_INFO("Gripper opened");
    GRIPPER_CONTROL_DELAY.sleep();

    // Detach the cube from the arm, and remove the cube from the scene.
    move_group_interface.detachObject("CUBE");
    planning_scene_interface.removeCollisionObjects({"CUBE"});

    // Lift-up Pose
    ROS_INFO("Planning and executing lift-up pose");
    switch_side_planner(move_group_interface);
    move_group_interface.setPoseTarget(
        message->pre_place_location, "arm_tcp_link"
    );
    if (planner_adapter(
            move_group_interface, planning_scene_storage,
            message->scene_name.data, PathSection::Untracked
        ) != 0) {
        ROS_ERROR("Failed to move to lift-up pose, exiting");
        return;
    }
    ROS_INFO("Lift-up pose executed");

    // Return gripper to neutral
    GRIPPER_CONTROL_DELAY.sleep();
    gripper_control_publisher.publish(gripper_neutral);
    ROS_INFO("Gripper returned to neutral state.");

    // Return to starting position
    {
        std::vector<double> joint_group_position;
        joint_group_position.resize(6, 0);
        switch_main_planner(move_group_interface);
        move_group_interface.setJointValueTarget(joint_group_position);
        if (planner_adapter(
                move_group_interface, planning_scene_storage,
                message->scene_name.data, PathSection::Home
            ) != 0) {
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