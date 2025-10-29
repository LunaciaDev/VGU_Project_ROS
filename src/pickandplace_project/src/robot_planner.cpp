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

    // construct the starting state from the message
    auto start_state = moveit::core::RobotState(move_group->getRobotModel());
    start_state.setVariablePositions(current_joint_configuration);
    move_group->setStartState(start_state);

    // assign the target pose
    move_group->setPoseTarget(target_pose, "arm_tcp_link");

    ROS_INFO("End effector %s", move_group -> getEndEffector().c_str());

    // plan
    const auto ok = static_cast<bool>(move_group->plan(plan));

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

    MoveGroupInterface       move_group(PLANNING_GROUP);

    const auto current_joint_configuration = request.joints_input.joints;
    MoveGroupInterface::Plan pre_grasp_pose, grasp_pose, pick_up_pose,
        place_pose;

    // Pre-Grasp
    {
        const auto result = plan_trajectory(
            &move_group, request.pick_pose, &current_joint_configuration[0]
        );
        const auto success = result.first;
        const auto plan = result.second;

        if (!success) {
            return false;
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
        const auto result = plan_trajectory(
            &move_group, lowered_pick_pose, current_joint_angles
        );
        const auto success = result.first;
        const auto plan = result.second;

        if (!success) {
            return false;
        }

        grasp_pose = plan;
    }

    current_joint_angles =
        grasp_pose.trajectory_.joint_trajectory.points.back().positions.data();

    // Pick-up
    {
        const auto result = plan_trajectory(
            &move_group, request.pick_pose, current_joint_angles
        );
        const auto success = result.first;
        const auto plan = result.second;

        if (!success) {
            return false;
        }

        pick_up_pose = plan;
    }

    current_joint_angles =
        pick_up_pose.trajectory_.joint_trajectory.points.back()
            .positions.data();

    // Place
    {
        const auto result = plan_trajectory(
            &move_group, request.place_pose, current_joint_angles
        );
        const auto success = result.first;
        const auto plan = result.second;

        if (!success) {
            return false;
        }

        place_pose = plan;
    }

    // If we reach this part, all planning section has succeeded.
    response.trajectories.push_back(pre_grasp_pose.trajectory_);
    response.trajectories.push_back(grasp_pose.trajectory_);
    response.trajectories.push_back(pick_up_pose.trajectory_);
    response.trajectories.push_back(place_pose.trajectory_);

    move_group.clearPoseTargets();

    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_planner");
    ros::NodeHandle          node_handle;

    const ros::ServiceServer service =
        node_handle.advertiseService("robot_planner_service", service_function);
    ROS_INFO("Service Ready");
    ros::spin();

    return 0;
}