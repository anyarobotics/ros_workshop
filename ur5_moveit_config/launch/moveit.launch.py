from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command


def load_yaml(path):
    with open(path, "r") as file:
        return yaml.safe_load(file)


def generate_launch_description():

    moveit_pkg = Path(
        get_package_share_directory("ur5_moveit_config")
    )

    description_pkg = Path(
        get_package_share_directory("ur5_description")
    )

    robot_xacro = description_pkg / "urdf" / "ur5.urdf.xacro"
    srdf_file = moveit_pkg / "config" / "ur5.srdf"

    kinematics_yaml = load_yaml(
        moveit_pkg / "config" / "kinematics.yaml"
    )

    ompl_yaml = load_yaml(
        moveit_pkg / "config" / "ompl_planning.yaml"
    )

    joint_limits_yaml = load_yaml(
        moveit_pkg / "config" / "joint_limits.yaml"
    )

    controllers_yaml = load_yaml(
        moveit_pkg / "config" / "moveit_controllers.yaml"
    )

    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                str(robot_xacro),
                " ur_type:=ur5",
            ]
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": srdf_file.read_text()
    }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            {"ompl": ompl_yaml},
            {"robot_description_planning": joint_limits_yaml},
            controllers_yaml,
            {
                "planning_pipelines": {
                    "pipeline_names": ["ompl"]
                }
            },
            {
                "plan_request_params": {
                    "planning_pipeline": "ompl",
                    "planning_id": "RRTConnectkConfigDefault",
                    "planning_time": 5.0,
                    "planning_attempts": 10,
                    "max_velocity_scaling_factor": 1.0,
                    "max_acceleration_scaling_factor": 1.0,
                }
            },
            {
                "use_sim_time": True,
                "moveit_manage_controllers": False,
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        move_group,
        rviz,
    ])
