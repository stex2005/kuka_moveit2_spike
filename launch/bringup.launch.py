"""Bring up KUKA kr_70_r2100 with MoveIt2 and ros2_control.

Usage:
  ros2 launch kuka_moveit2_spike bringup.launch.py              # headless
  ros2 launch kuka_moveit2_spike bringup.launch.py gui:=true     # with RViz
"""

import os

from ament_index_python.packages import get_package_share_directory  # type: ignore
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument  # type: ignore
from launch.conditions import IfCondition  # type: ignore
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _build_moveit_config(pkg_share):
    return (
        MoveItConfigsBuilder(
            robot_name="kr_70_r2100", package_name="kuka_moveit2_spike"
        )
        .robot_description(
            file_path=os.path.join(pkg_share, "urdf", "kr_70_r2100.urdf.xacro"),
            mappings={"use_fake_hardware": "true"},
        )
        .robot_description_semantic(
            file_path=os.path.join(pkg_share, "srdf", "kr_70_r2100.srdf.xacro")
        )
        .robot_description_kinematics(
            file_path=os.path.join(pkg_share, "config", "kinematics.yaml")
        )
        .joint_limits(file_path=os.path.join(pkg_share, "config", "joint_limits.yaml"))
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl",
        )
        .trajectory_execution(
            file_path=os.path.join(pkg_share, "config", "moveit_controllers.yaml")
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )


def generate_launch_description():
    pkg_share = get_package_share_directory("kuka_moveit2_spike")
    moveit_config = _build_moveit_config(pkg_share)
    gui = LaunchConfiguration("gui")

    rviz_config = os.path.join(pkg_share, "config", "moveit.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_fake_hardware", default_value="true"),
            DeclareLaunchArgument(
                "gui", default_value="false", description="Launch RViz"
            ),
            # robot_state_publisher
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[moveit_config.robot_description],
            ),
            # ros2_control
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    moveit_config.robot_description,
                    os.path.join(pkg_share, "config", "ros2_controllers.yaml"),
                ],
                output="screen",
            ),
            # Controller spawners
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_trajectory_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
            ),
            # MoveIt2 move_group
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[moveit_config.to_dict(), {"use_sim_time": False}],
            ),
            # RViz (conditional)
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                condition=IfCondition(gui),
                arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                ],
            ),
        ]
    )
