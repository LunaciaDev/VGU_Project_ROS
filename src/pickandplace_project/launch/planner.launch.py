from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        robot_name="ur10e_rg2", package_name="robot_moveit"
    ).to_moveit_configs()

    robot_planner_node = Node(
        package="pickandplace_project",
        executable="robot_planner",
        name="robot_planner_wrapper",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([robot_planner_node])
