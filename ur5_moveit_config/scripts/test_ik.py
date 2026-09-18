#!/usr/bin/env python3

from pathlib import Path

import rclpy
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy


def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def main():
    rclpy.init()

    description_pkg = get_package_share_directory("ur5_description")
    moveit_pkg = get_package_share_directory("ur5_moveit_config")

    urdf_path = Path(description_pkg) / "urdf" / "ur5.urdf.xacro"
    srdf_path = Path(moveit_pkg) / "config" / "ur5.srdf"
    kinematics_path = Path(moveit_pkg) / "config" / "kinematics.yaml"
    joint_limits_path = Path(moveit_pkg) / "config" / "joint_limits.yaml"
    ompl_path = Path(moveit_pkg) / "config" / "ompl_planning.yaml"

    urdf_xml = xacro.process_file(
        str(urdf_path),
        mappings={"ur_type": "ur5"},
    ).toxml()

    srdf_xml = srdf_path.read_text()

    config = {
        "use_sim_time": True,
        "robot_description": urdf_xml,
        "robot_description_semantic": srdf_xml,
        "robot_description_kinematics": load_yaml(kinematics_path),
        "robot_description_planning": load_yaml(joint_limits_path),

        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/attached_collision_object",
            "monitored_planning_scene_topic": "/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },

        "planning_pipelines": {
            "pipeline_names": ["ompl"],
        },

        "default_planning_pipeline": "ompl",

        "ompl": load_yaml(ompl_path),

        "plan_request_params": {
            "planning_attempts": 20,
            "planning_time": 10.0,
            "planning_pipeline": "ompl",
            "max_velocity_scaling_factor": 0.2,
            "max_acceleration_scaling_factor": 0.2,
        },
    }

    robot = MoveItPy(
        node_name="test_ik",
        config_dict=config,
    )

    arm = robot.get_planning_component("ur_manipulator")

    pose = PoseStamped()
    pose.header.frame_id = "world"

    pose.pose.position.x = 0.40
    pose.pose.position.y = 0.00
    pose.pose.position.z = 0.72

    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 1.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.0

    print("\nTarget:")
    print("  position: 0.40, 0.00, 0.72")
    print("  orientation: 0, 1, 0, 0")

    arm.set_start_state_to_current_state()

    arm.set_goal_state(
        pose_stamped_msg=pose,
        pose_link="tool0",
    )

    print("\nAttempting IK/planning...")

    plan = arm.plan()

    if plan:
        print("\nSUCCESS: target pose is reachable.")
    else:
        print("\nFAILURE: MoveIt could not find a solution.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
