from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def load_yaml(path):
    with open(path, "r") as file:
        return yaml.safe_load(file)


def generate_launch_description():

    description_pkg = Path(
        get_package_share_directory("ur5_description")
    )

    moveit_pkg = Path(
        get_package_share_directory("ur5_moveit_config")
    )

    urdf_path = description_pkg / "urdf" / "ur5.urdf.xacro"
    srdf_path = moveit_pkg / "config" / "ur5.srdf"
    kinematics_path = moveit_pkg / "config" / "kinematics.yaml"
    ompl_path = moveit_pkg / "config" / "ompl_planning.yaml"
    joint_limits_path = moveit_pkg / "config" / "joint_limits.yaml"

    kinematics_yaml = load_yaml(kinematics_path)
    ompl_yaml = load_yaml(ompl_path)
    joint_limits_yaml = load_yaml(joint_limits_path)

    robot_description = ParameterValue(
        Command([
            "xacro ",
            str(urdf_path),
            " ur_type:=ur5",
        ]),
        value_type=str,
    )

    robot_description_semantic = srdf_path.read_text()

    planning_pipelines_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl"],
        "ompl": ompl_yaml,
    }

    test_node = Node(
        package="ur5_moveit_config",
        executable="test_scan_pose.py",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "robot_description_semantic": robot_description_semantic,
                "robot_description_kinematics": kinematics_yaml,
                "robot_description_planning": joint_limits_yaml,
            },
            planning_pipelines_config,
            {
                "plan_request_params": {
                    "planning_pipeline": "ompl",
                    "planner_id": "RRTConnectkConfigDefault",
                    "planning_attempts": 10,
                    "planning_time": 5.0,
                    "max_velocity_scaling_factor": 1.0,
                    "max_acceleration_scaling_factor": 1.0,
                }
            },
            {
                "use_sim_time": True,
            },
        ],
    )

    return LaunchDescription([
        test_node,
    ])