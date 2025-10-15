from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        robot_name="ur10e_rg2", package_name="robot_moveit"
    ).to_moveit_configs()

    ros_tcp_endpoint = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        output="screen",
        arguments="--wait",
        parameters=[{
            'tcp_ip': "0.0.0.0",
            'tcp_port': "10000"
        }]
    )

    robot_planner_node = Node(
        package="pickandplace_project",
        executable="robot_planner",
        name="robot_planner_wrapper",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([ros_tcp_endpoint, robot_planner_node])
