#include <memory>

#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/utilities.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <utility>
#include "geometry_msgs/msg/pose.hpp"
#include "ros_unity_integration/srv/integration_service.hpp"

auto plan_trajectory(moveit::planning_interface::MoveGroupInterface* move_group,
                     ros_unity_integration::srv::IntegrationService_Request::_pick_pose_type target_pose,
                     // Man do I miss Python's duck typing
                     double* current_joint_configuration)
{
    auto plan = moveit::planning_interface::MoveGroupInterface::Plan();

    // Construct the starting state from the message
    auto start_state = moveit::core::RobotState(move_group->getRobotModel());
    start_state.setVariablePositions(current_joint_configuration);
    move_group->setStartState(start_state);

    // Assign the target pose
    move_group->setPoseTarget(target_pose);

    // Plan!
    const auto ok = static_cast<bool>(move_group->plan(plan));

    return std::make_pair(ok, plan);
}

auto callback(const std::shared_ptr<rclcpp::Node> node,
              const std::shared_ptr<ros_unity_integration::srv::IntegrationService_Request> request,
              const std::shared_ptr<ros_unity_integration::srv::IntegrationService_Response> response)
{
    using namespace moveit::planning_interface;

    auto move_group = MoveGroupInterface(node, "robot_arm");
    auto current_joint_configuration = request->joints_input.joints;
    MoveGroupInterface::Plan pre_grasp_pose, grasp_pose, pick_up_pose, place_pose;

    // Pre-Grasp
    {
        const auto [success, plan] = plan_trajectory(&move_group, request->pick_pose, &current_joint_configuration[0]);

        if (!success)
        {
            return response;
        }

        pre_grasp_pose = plan;
    }

    auto current_joint_angles = pre_grasp_pose.trajectory.joint_trajectory.points.back().positions.data();
    auto lowered_pick_pose = geometry_msgs::msg::Pose(request->pick_pose);
    lowered_pick_pose.position.set__z(lowered_pick_pose.position.z - 0.05);

    // Grasp
    {
        const auto [success, plan] = plan_trajectory(&move_group, lowered_pick_pose, current_joint_angles);

        if (!success)
        {
            return response;
        }

        grasp_pose = plan;
    }

    current_joint_angles = grasp_pose.trajectory.joint_trajectory.points.back().positions.data();

    // Pick-up
    {
        const auto [success, plan] = plan_trajectory(&move_group, request->pick_pose, current_joint_angles);

        if (!success)
        {
            return response;
        }

        pick_up_pose = plan;
    }

    current_joint_angles = pick_up_pose.trajectory.joint_trajectory.points.back().positions.data();

    // Place
    {
        const auto [success, plan] = plan_trajectory(&move_group, request->place_pose, current_joint_angles);

        if (!success)
        {
            return response;
        }

        place_pose = plan;
    }

    // If we reach this part, all planning section has succeeded.
    response->trajectories.push_back(pre_grasp_pose.trajectory);
    response->trajectories.push_back(grasp_pose.trajectory);
    response->trajectories.push_back(pick_up_pose.trajectory);
    response->trajectories.push_back(place_pose.trajectory);

    move_group.clearPoseTargets();

    return response;
}

int main(int argc, char* argv[])
{
    // Initialize ROS and Node
    rclcpp::init(argc, argv);

    // Create a ROS logger
    const auto logger = rclcpp::get_logger("robot_planner");
    const auto node = std::make_shared<rclcpp::Node>("robot_planner");

    // Create a ROS Service
    rclcpp::Service<ros_unity_integration::srv::IntegrationService>::SharedPtr service =
        node->create_service<ros_unity_integration::srv::IntegrationService>(
            "robot_planner_service",
            [node](const std::shared_ptr<ros_unity_integration::srv::IntegrationService_Request> request,
                   const std::shared_ptr<ros_unity_integration::srv::IntegrationService_Response> response) {
                return callback(node, request, response);
            });

    // Log that the service is created
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot Planner Service started.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}