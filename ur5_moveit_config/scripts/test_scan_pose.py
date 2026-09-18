#!/usr/bin/env python3

from pathlib import Path

import rclpy
import yaml
import xacro
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy


def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def main(args=None):
    rclpy.init(args=args)

    # ------------------------------------------------------------
    # Package paths
    # ------------------------------------------------------------
    description_pkg = get_package_share_directory("ur5_description")
    moveit_pkg = get_package_share_directory("ur5_moveit_config")

    urdf_path = (
        Path(description_pkg)
        / "urdf"
        / "ur5.urdf.xacro"
    )

    srdf_path = (
        Path(moveit_pkg)
        / "config"
        / "ur5.srdf"
    )

    kinematics_path = (
        Path(moveit_pkg)
        / "config"
        / "kinematics.yaml"
    )

    joint_limits_path = (
        Path(moveit_pkg)
        / "config"
        / "joint_limits.yaml"
    )

    ompl_path = (
        Path(moveit_pkg)
        / "config"
        / "ompl_planning.yaml"
    )

    # ------------------------------------------------------------
    # Xacro -> URDF
    # ------------------------------------------------------------
    urdf_xml = xacro.process_file(
        str(urdf_path),
        mappings={
            "ur_type": "ur5",
        },
    ).toxml()

    # ------------------------------------------------------------
    # Load configuration files
    # ------------------------------------------------------------
    srdf_xml = srdf_path.read_text()

    kinematics = load_yaml(kinematics_path)
    joint_limits = load_yaml(joint_limits_path)
    ompl_config = load_yaml(ompl_path)

    # ------------------------------------------------------------
    # MoveItPy configuration
    # ------------------------------------------------------------
    moveit_config = {
        # CRITICAL for Gazebo
        "use_sim_time": True,

        # Robot description
        "robot_description": urdf_xml,

        # Semantic description
        "robot_description_semantic": srdf_xml,

        # Kinematics
        "robot_description_kinematics": kinematics,

        # Joint limits
        "robot_description_planning": joint_limits,

        # Planning scene monitor
        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/attached_collision_object",
            "monitored_planning_scene_topic": "/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },

        # Planning pipeline
        "planning_pipelines": {
            "pipeline_names": ["ompl"],
        },

        "default_planning_pipeline": "ompl",

        # OMPL
        "ompl": ompl_config,

        # Planning request
        "plan_request_params": {
            "planning_attempts": 10,
            "planning_time": 5.0,
            "planning_pipeline": "ompl",
            "max_velocity_scaling_factor": 0.2,
            "max_acceleration_scaling_factor": 0.2,
        },

        

        "moveit_controller_manager":
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",

        "moveit_simple_controller_manager": {
            "controller_names": [
                "joint_trajectory_controller"
            ],

            "joint_trajectory_controller": {
                "type": "FollowJointTrajectory",
                "joints": [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint"
                ],
                "action_ns": "follow_joint_trajectory"
            }
        },
    }

    # ------------------------------------------------------------
    # Start MoveItPy
    # ------------------------------------------------------------
    robot = MoveItPy(
        node_name="scan_pose_moveit",
        config_dict=moveit_config,
    )

    print("MoveItPy initialized")

    # ------------------------------------------------------------
    # Planning group
    # ------------------------------------------------------------
    arm = robot.get_planning_component("ur_manipulator")

    # ------------------------------------------------------------
    # Reference scan pose
    # ------------------------------------------------------------
    pose_goal = PoseStamped()

    pose_goal.header.frame_id = "world"

    pose_goal.pose.position.x = 0.40
    pose_goal.pose.position.y = 0.00
    pose_goal.pose.position.z = 0.72

    pose_goal.pose.orientation.x = 0.0
    pose_goal.pose.orientation.y = 1.0
    pose_goal.pose.orientation.z = 0.0
    pose_goal.pose.orientation.w = 0.0

    print(
        "Planning to scan pose: "
        "(x=0.40, y=0.00, z=0.72)"
    )

    # ------------------------------------------------------------
    # Current robot state
    # ------------------------------------------------------------
    time.sleep(1.0)
    arm.set_start_state_to_current_state()

    # ------------------------------------------------------------
    # Goal
    # ------------------------------------------------------------
    arm.set_goal_state(
        pose_stamped_msg=pose_goal,
        pose_link="tool0",
    )

    # ------------------------------------------------------------
    # Plan
    # ------------------------------------------------------------
    print("Planning...")

    plan = arm.plan()

    if not plan:
        print("Planning failed")
        rclpy.shutdown()
        return

    print("Plan successful. Executing...")

    # ------------------------------------------------------------
    # Execute
    # ------------------------------------------------------------
    robot.execute(
        plan.trajectory,
        controllers=["joint_trajectory_controller"],
    )

    print("Execution complete")

    rclpy.shutdown()


if __name__ == "__main__":
    main()