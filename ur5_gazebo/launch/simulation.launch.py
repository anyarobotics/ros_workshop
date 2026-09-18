from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    description_pkg = Path(
        get_package_share_directory("ur5_description")
    )

    gazebo_pkg = Path(
        get_package_share_directory("ur5_gazebo")
    )

    # ------------------------------------------------------------
    # Package paths
    # ------------------------------------------------------------

    robot_xacro = (
        description_pkg
        / "urdf"
        / "ur5.urdf.xacro"
    )

    controllers_file = (
        gazebo_pkg
        / "config"
        / "controllers.yaml"
    )

    camera_bridge_file = (
        gazebo_pkg
        / "config"
        / "camera_bridge.yaml"
    )

    world_file = (
        gazebo_pkg
        / "worlds"
        / "pick_world.sdf"
    )

    # ------------------------------------------------------------
    # Gazebo resource path
    #
    # Gazebo needs the parent directory containing:
    #
    #   robotiq_description/
    #
    # so that:
    #
    # model://robotiq_description/...
    #
    # resolves correctly.
    # ------------------------------------------------------------

    robotiq_share = Path(
        get_package_share_directory(
            "robotiq_description"
        )
    )

    robotiq_resource_path = str(
        robotiq_share.parent
    )

    existing_resource_path = os.environ.get(
        "GZ_SIM_RESOURCE_PATH",
        ""
    )

    if existing_resource_path:

        gazebo_resource_path = (
            robotiq_resource_path
            + ":"
            + existing_resource_path
        )

    else:

        gazebo_resource_path = (
            robotiq_resource_path
        )

    # ------------------------------------------------------------
    # Simulation time
    # ------------------------------------------------------------

    use_sim_time = LaunchConfiguration(
        "use_sim_time"
    )

    # ------------------------------------------------------------
    # Robot description
    # ------------------------------------------------------------

    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                str(robot_xacro),
                " ur_type:=ur5",
                " simulation_controllers:=",
                str(controllers_file),
            ]
        )
    }

    # ------------------------------------------------------------
    # Gazebo
    # ------------------------------------------------------------

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                str(
                    get_package_share_directory(
                        "ros_gz_sim"
                    )
                ),
                "/launch/gz_sim.launch.py",
            ]
        ),
        launch_arguments={
            "gz_args": "-r " + str(world_file),
        }.items(),
    )

    # ------------------------------------------------------------
    # Robot State Publisher
    # ------------------------------------------------------------

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {
                "use_sim_time": use_sim_time
            },
        ],
    )

    # ------------------------------------------------------------
    # Spawn robot
    # ------------------------------------------------------------

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "ur5_pick_place",
            "-allow_renaming",
        ],
    )

    # ------------------------------------------------------------
    # Camera + /clock bridge
    # ------------------------------------------------------------

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            "config_file:="
            + str(camera_bridge_file),
        ],
        output="screen",
    )

    # ------------------------------------------------------------
    # Joint state broadcaster
    # ------------------------------------------------------------

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # ------------------------------------------------------------
    # UR5 trajectory controller
    # ------------------------------------------------------------

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # ------------------------------------------------------------
    # Robotiq gripper controller
    # ------------------------------------------------------------

    robotiq_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robotiq_gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # ------------------------------------------------------------
    # Launch description
    # ------------------------------------------------------------

    return LaunchDescription(
        [

            # Make Robotiq meshes available to Gazebo
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=gazebo_resource_path,
            ),

            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
            ),

            gazebo,

            robot_state_publisher,

            spawn_robot,

            camera_bridge,

            joint_state_broadcaster_spawner,

            joint_trajectory_controller_spawner,

            robotiq_gripper_controller_spawner,
        ]
    )