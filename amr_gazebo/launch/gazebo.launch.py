"""
Usage:
    ros2 launch amr_gazebo gazebo.launch.py
    ros2 launch amr_gazebo gazebo.launch.py world:=$(ros2 pkg prefix amr_gazebo)/share/amr_gazebo/worlds/amr_world.sdf
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    amr_gazebo_share = get_package_share_directory("amr_gazebo")
    amr_description_share = get_package_share_directory("amr_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    default_model_path = os.path.join(amr_gazebo_share, "urdf", "amr_gazebo.urdf")
    default_world_path = os.path.join(amr_gazebo_share, "worlds", "factory_world.sdf")
    default_bridge_config_path = os.path.join(amr_gazebo_share, "config", "amr_bridge.yaml")
    default_rviz_config_path = os.path.join(amr_gazebo_share, "rviz", "amr_gazebo.rviz")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=default_model_path,
        description="Absolute path to the AMR Gazebo urdf/xacro file",
    )
    world_arg = DeclareLaunchArgument(
        name="world",
        default_value=default_world_path,
        description="Absolute path to the Gazebo world SDF file "
        "(defaults to the factory world; pass amr_world.sdf's path for an empty world)",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use Gazebo's simulation clock (bridged from /clock)",
    )
    use_rviz_arg = DeclareLaunchArgument(
        name="use_rviz",
        default_value="true",
        description="Launch RViz alongside Gazebo",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    set_gz_resource_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.dirname(amr_description_share),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [LaunchConfiguration("world"), " -r"],
        }.items(),
    )

    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "amr",
            "-z", "0.05",
        ],
        output="screen",
    )
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={default_bridge_config_path}"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )
    image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["camera_left/image", "camera_right/image"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", default_rviz_config_path],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            model_arg,
            world_arg,
            use_sim_time_arg,
            use_rviz_arg,
            set_gz_resource_path,
            robot_state_publisher_node,
            gz_sim,
            spawn_node,
            bridge_node,
            image_bridge_node,
            rviz_node,
        ]
    )
