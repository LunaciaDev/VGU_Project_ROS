#include "moveit/move_group_interface/move_group_interface.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/service_server.h"
#include "ros_unity_integration/IntegrationService.h"

using IntegrationService = ros_unity_integration::IntegrationService;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

std::pair<bool, MoveGroupInterface::Plan> plan_trajectory(
    MoveGroupInterface*                              move_group,
    const geometry_msgs::Pose_<std::allocator<void>> target_pose,
    const double*                                    current_joint_configuration
) {
    auto plan = MoveGroupInterface::Plan();

    ROS_INFO("Constructing starting state of the robot");

    // construct the starting state from the message
    auto start_state = moveit::core::RobotState(move_group->getRobotModel());
    start_state.setVariablePositions(current_joint_configuration);
    move_group->setStartState(start_state);

    ROS_INFO("Constructing target position of the robot");

    // assign the target pose
    move_group->setPoseTarget(target_pose, "arm_tcp_link");

    ROS_INFO("Passing to planner...");

    // plan
    const auto ok =
        (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    ROS_INFO("Planner finished");

    return std::make_pair(ok, plan);
}

bool service_function(
    // ROS doesnt like these being const for some reason
    // Maybe they are template that get some stuff injected in comptime that
    // make those not const-able?
    IntegrationService::Request&  request,
    IntegrationService::Response& response
) {
    static const std::string PLANNING_GROUP = "robot_arm";

    ROS_INFO("Received request from Unity.");

    MoveGroupInterface move_group(PLANNING_GROUP);

    const auto current_joint_configuration = request.joints_input.joints;
    MoveGroupInterface::Plan pre_grasp_pose, grasp_pose, pick_up_pose,
        place_pose;

    // Pre-Grasp
    {
        ROS_INFO("Planning trajectory for pre_grasp");

        const auto result = plan_trajectory(
            &move_group, request.pick_pose, &current_joint_configuration[0]
        );
        const auto success = result.first;
        const auto plan = result.second;

        if (!success) {
            ROS_ERROR("Failed to calculate plan for pre_grasp");
            /*
                Great ROS1 design by the way.
                We have to return *something* back to the caller, otherwise they
               are left hanging. Guess what returning false does? It send
               nothing back to the caller!
            */
            return true;
        }

        pre_grasp_pose = plan;
    }

    auto current_joint_angles =
        pre_grasp_pose.trajectory_.joint_trajectory.points.back()
            .positions.data();
    auto lowered_pick_pose = geometry_msgs::Pose(request.pick_pose);
    lowered_pick_pose.position.z = lowered_pick_pose.position.z - 0.05;

    // Grasp
    {
        ROS_INFO("Planning trajectory for grasp");

        const auto result = plan_trajectory(
            &move_group, lowered_pick_pose, current_joint_angles
        );
        const auto success = result.first;
        const auto plan = result.second;

        ROS_INFO("Planner finished");

        if (!success) {
            ROS_ERROR("Failed to calculate plan for grasp");
            return true;
        }

        grasp_pose = plan;
    }

    current_joint_angles =
        grasp_pose.trajectory_.joint_trajectory.points.back().positions.data();

    // Pick-up
    {
        ROS_INFO("Planning trajectory for pick_up");

        const auto result = plan_trajectory(
            &move_group, request.pick_pose, current_joint_angles
        );
        const auto success = result.first;
        const auto plan = result.second;

        ROS_INFO("Planner finished");

        if (!success) {
            ROS_ERROR("Failed to calculate plan for pick_up");
            return true;
        }

        pick_up_pose = plan;
    }

    current_joint_angles =
        pick_up_pose.trajectory_.joint_trajectory.points.back()
            .positions.data();

    // Place
    {
        ROS_INFO("Planning trajectory for place");

        const auto result = plan_trajectory(
            &move_group, request.place_pose, current_joint_angles
        );
        const auto success = result.first;
        const auto plan = result.second;

        ROS_INFO("Planner finished");

        if (!success) {
            ROS_ERROR("Failed to calculate plan for place");
            return true;
        }

        place_pose = plan;
    }

    // If we reach this part, all planning section has succeeded.
    ROS_INFO(
        "All trajectories successfully planned. Constructing the return "
        "message."
    );

    response.trajectories.push_back(pre_grasp_pose.trajectory_);
    response.trajectories.push_back(grasp_pose.trajectory_);
    response.trajectories.push_back(pick_up_pose.trajectory_);
    response.trajectories.push_back(place_pose.trajectory_);

    move_group.clearPoseTargets();

    ROS_INFO("Sending plans back for Unity to execute");

    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_planner");
    ros::NodeHandle   node_handle;

    // why..?
    ros::AsyncSpinner spinner = ros::AsyncSpinner(2);

    spinner.start();
    const ros::ServiceServer service =
        node_handle.advertiseService("robot_planner_service", service_function);
    ROS_INFO("robot_planner_service ready");
    ros::waitForShutdown();

    return 0;
}