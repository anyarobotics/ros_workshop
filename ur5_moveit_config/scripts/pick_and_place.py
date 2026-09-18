#!/usr/bin/env python3

from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from control_msgs.action import ParallelGripperCommand
from geometry_msgs.msg import PoseStamped, Pose
from moveit.planning import MoveItPy
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from rclpy.action import ActionClient
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener, TransformException


# ============================================================
# TASK GEOMETRY
# ============================================================

SCAN_POSITIONS = [
    (0.40, 0.00),
    (0.40, 0.08),
    (0.40, -0.08),
    (0.45, 0.00),
    (0.35, 0.00),
    (0.45, 0.08),
    (0.35, -0.08),
    (0.40, 0.04),
    (0.40, -0.04),
    (0.45, -0.08),
]

RED_BIN = (0.20, 0.40)
BLUE_BIN = (0.20, -0.40)

EXPECTED_BOXES = 4
MAX_SCAN_CYCLES = 3

# Tabletop surface in world frame
TABLE_Z = 0.50

TABLE_X_MIN = 0.25
TABLE_X_MAX = 0.55
TABLE_Y_MIN = -0.15
TABLE_Y_MAX = 0.15

# Pick/place heights
PRE_GRASP_Z = 0.72
GRASP_Z = 0.68
LIFT_Z = 0.80

# Tool orientation
QX = 0.0
QY = 1.0
QZ = 0.0
QW = 0.0


# ============================================================
# GRIPPER
# ============================================================

GRIPPER_ACTION = "/robotiq_gripper_controller/gripper_cmd"
GRIPPER_JOINT = "robotiq_85_left_knuckle_joint"

GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.7929
GRIPPER_EFFORT = 40.0


# ============================================================
# ARM CONTROLLER
# ============================================================

ARM_CONTROLLER = "joint_trajectory_controller"


# ============================================================
# PERCEPTION
# ============================================================

class Perception(Node):

    def __init__(self):

        super().__init__("pick_place_perception")

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.detections = []

        # TF
        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # Camera info
        self.create_subscription(
            CameraInfo,
            "/wrist_camera/camera_info",
            self.camera_info_callback,
            10
        )

        # Camera image
        self.create_subscription(
            Image,
            "/wrist_camera/image_raw",
            self.image_callback,
            10
        )

        self.get_logger().info(
            "Perception node started"
        )

    # --------------------------------------------------------
    # Camera calibration
    # --------------------------------------------------------

    def camera_info_callback(self, msg):

        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    # --------------------------------------------------------
    # Image callback
    # --------------------------------------------------------

    def image_callback(self, msg):

        if self.fx is None:
            return

        if msg.encoding not in ("rgb8", "bgr8"):
            return

        image = np.frombuffer(
            msg.data,
            dtype=np.uint8
        ).reshape(
            msg.height,
            msg.width,
            3
        )

        if msg.encoding == "rgb8":

            image = cv2.cvtColor(
                image,
                cv2.COLOR_RGB2BGR
            )

        hsv = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2HSV
        )

        # ----------------------------------------------------
        # RED
        # ----------------------------------------------------

        red_mask = cv2.inRange(
            hsv,
            np.array([0, 120, 70]),
            np.array([10, 255, 255])
        )

        red_mask |= cv2.inRange(
            hsv,
            np.array([170, 120, 70]),
            np.array([180, 255, 255])
        )

        # ----------------------------------------------------
        # BLUE
        # ----------------------------------------------------

        blue_mask = cv2.inRange(
            hsv,
            np.array([90, 100, 50]),
            np.array([140, 255, 255])
        )

        detections = []

        for mask, color in (
            (red_mask, "red"),
            (blue_mask, "blue"),
        ):

            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:

                area = cv2.contourArea(contour)

                if not (100.0 < area < 15000.0):
                    continue

                moments = cv2.moments(contour)

                if moments["m00"] == 0:
                    continue

                u = (
                    moments["m10"]
                    / moments["m00"]
                )

                v = (
                    moments["m01"]
                    / moments["m00"]
                )

                point = self.pixel_to_world(
                    u,
                    v
                )

                if point is None:
                    continue

                x, y, z = point

                # Only accept points inside the usable
                # detection area of our table.
                if (
                    TABLE_X_MIN <= x <= TABLE_X_MAX
                    and
                    TABLE_Y_MIN <= y <= TABLE_Y_MAX
                ):

                    detections.append(
                        (color, x, y)
                    )

        self.detections = detections

    # --------------------------------------------------------
    # Pixel -> world
    # --------------------------------------------------------

    def pixel_to_world(self, u, v):

        ray_camera = np.array([
            (u - self.cx) / self.fx,
            (v - self.cy) / self.fy,
            1.0
        ])

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    "world",
                    "wrist_camera_optical_link",
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(
                        seconds=0.1
                    )
                )
            )

        except TransformException:

            return None

        # Camera position
        t = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])

        # Quaternion
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        # Quaternion -> rotation matrix
        R = np.array([
            [
                1 - 2 * (qy*qy + qz*qz),
                2 * (qx*qy - qz*qw),
                2 * (qx*qz + qy*qw)
            ],
            [
                2 * (qx*qy + qz*qw),
                1 - 2 * (qx*qx + qz*qz),
                2 * (qy*qz - qx*qw)
            ],
            [
                2 * (qx*qz - qy*qw),
                2 * (qy*qz + qx*qw),
                1 - 2 * (qx*qx + qy*qy)
            ]
        ])

        # Camera ray -> world ray
        ray_world = R @ ray_camera

        if abs(ray_world[2]) < 1e-9:
            return None

        # Intersect with tabletop plane
        distance = (
            TABLE_Z - t[2]
        ) / ray_world[2]

        if distance <= 0:
            return None

        point = t + distance * ray_world

        return (
            float(point[0]),
            float(point[1]),
            float(point[2])
        )

    # --------------------------------------------------------
    # Collect multiple detections
    # --------------------------------------------------------

    def collect_boxes(
        self,
        duration_sec=2.5
    ):

        samples = []

        end_time = (
            time.monotonic()
            + duration_sec
        )

        while time.monotonic() < end_time:

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            samples.extend(
                self.detections
            )

        clusters = []

        for color, x, y in samples:

            matched = None

            for cluster in clusters:

                if cluster["color"] != color:
                    continue

                distance = np.hypot(
                    cluster["x"] - x,
                    cluster["y"] - y
                )

                if distance < 0.05:

                    matched = cluster
                    break

            if matched is None:

                clusters.append({
                    "color": color,
                    "x": x,
                    "y": y,
                    "readings": 1
                })

            else:

                n = matched["readings"] + 1

                matched["x"] = (
                    matched["x"] *
                    matched["readings"]
                    + x
                ) / n

                matched["y"] = (
                    matched["y"] *
                    matched["readings"]
                    + y
                ) / n

                matched["readings"] = n

        return [
            box
            for box in clusters
            if box["readings"] >= 3
        ]

    # --------------------------------------------------------
    # Refine selected object
    # --------------------------------------------------------

    def refine_box(
        self,
        color,
        target_x,
        target_y,
        duration_sec=1.5
    ):

        readings = []

        end_time = (
            time.monotonic()
            + duration_sec
        )

        while time.monotonic() < end_time:

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            for detected_color, x, y in self.detections:

                if detected_color != color:
                    continue

                if abs(x - target_x) >= 0.05:
                    continue

                if abs(y - target_y) >= 0.05:
                    continue

                readings.append(
                    (x, y)
                )

        if len(readings) < 3:
            return None

        x = sum(
            p[0]
            for p in readings
        ) / len(readings)

        y = sum(
            p[1]
            for p in readings
        ) / len(readings)

        return x, y


# ============================================================
# MOVEIT + GRIPPER
# ============================================================

class PickPlace:

    def __init__(self, node):

        self.node = node

        description_pkg = (
            get_package_share_directory(
                "ur5_description"
            )
        )

        moveit_pkg = (
            get_package_share_directory(
                "ur5_moveit_config"
            )
        )

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

        with open(
            kinematics_path,
            "r"
        ) as f:
            kinematics = yaml.safe_load(f)

        with open(
            joint_limits_path,
            "r"
        ) as f:
            joint_limits = yaml.safe_load(f)

        with open(
            ompl_path,
            "r"
        ) as f:
            ompl_config = yaml.safe_load(f)

        urdf_xml = xacro.process_file(
            str(urdf_path),
            mappings={
                "ur_type": "ur5"
            }
        ).toxml()

        srdf_xml = srdf_path.read_text()

        moveit_config = {

            "use_sim_time": True,

            "robot_description":
                urdf_xml,

            "robot_description_semantic":
                srdf_xml,

            "robot_description_kinematics":
                kinematics,

            "robot_description_planning":
                joint_limits,

            "planning_scene_monitor_options": {

                "name":
                    "planning_scene_monitor",

                "robot_description":
                    "robot_description",

                "joint_state_topic":
                    "/joint_states",

                "attached_collision_object_topic":
                    "/attached_collision_object",

                "monitored_planning_scene_topic":
                    "/monitored_planning_scene",

                "wait_for_initial_state_timeout":
                    10.0,
            },

            "planning_pipelines": {
                "pipeline_names":
                    ["ompl"]
            },

            "default_planning_pipeline":
                "ompl",

            "ompl":
                ompl_config,

            "plan_request_params": {

                "planning_attempts":
                    20,

                "planning_time":
                    10.0,

                "planning_pipeline":
                    "ompl",

                "max_velocity_scaling_factor":
                    0.10,

                "max_acceleration_scaling_factor":
                    0.10,
            },

            "moveit_controller_manager":
                "moveit_simple_controller_manager/"
                "MoveItSimpleControllerManager",

            "moveit_simple_controller_manager": {

                "controller_names": [
                    ARM_CONTROLLER
                ],

                ARM_CONTROLLER: {

                    "type":
                        "FollowJointTrajectory",

                    "joints": [

                        "shoulder_pan_joint",
                        "shoulder_lift_joint",
                        "elbow_joint",
                        "wrist_1_joint",
                        "wrist_2_joint",
                        "wrist_3_joint",
                    ],

                    "action_ns":
                        "follow_joint_trajectory",
                },
            },
        }

        # ----------------------------------------------------
        # Start MoveItPy
        # ----------------------------------------------------

        self.robot = MoveItPy(
            node_name="pick_place_moveit",
            config_dict=moveit_config
        )

        self.arm = (
            self.robot.get_planning_component(
                "ur_manipulator"
            )
        )

        # ----------------------------------------------------
        # Planning scene monitor
        # ----------------------------------------------------

        self.planning_scene_monitor = (
            self.robot.get_planning_scene_monitor()
        )

        # Add the real Gazebo table to MoveIt's world model.
        self.add_table_to_planning_scene()

        # ----------------------------------------------------
        # Gripper action client
        # ----------------------------------------------------

        self.gripper = ActionClient(
            node,
            ParallelGripperCommand,
            GRIPPER_ACTION
        )

        self.node.get_logger().info(
            "MoveItPy initialized"
        )

    # ========================================================
    # ADD TABLE TO MOVEIT PLANNING SCENE
    # ========================================================

    def add_table_to_planning_scene(self):

        with self.planning_scene_monitor.read_write() as scene:

            # ------------------------------------------------
            # TABLE TOP
            #
            # Gazebo:
            # model center = (0.7, 0, 0)
            # top collision center = z=0.475
            # size = 0.8 x 0.8 x 0.05
            # ------------------------------------------------

            table = CollisionObject()

            table.header.frame_id = "world"
            table.id = "table_top"

            primitive = SolidPrimitive()

            primitive.type = SolidPrimitive.BOX

            primitive.dimensions = [
                0.8,
                0.8,
                0.05
            ]

            pose = Pose()

            pose.position.x = 0.7
            pose.position.y = 0.0
            pose.position.z = 0.475

            pose.orientation.w = 1.0

            table.primitives.append(
                primitive
            )

            table.primitive_poses.append(
                pose
            )

            table.operation = CollisionObject.ADD

            scene.apply_collision_object(
                table
            )

            # ------------------------------------------------
            # TABLE LEGS
            # ------------------------------------------------

            leg_positions = [
                (1.05,  0.35),
                (1.05, -0.35),
                (0.35,  0.35),
                (0.35, -0.35),
            ]

            for index, (x, y) in enumerate(
                leg_positions
            ):

                leg = CollisionObject()

                leg.header.frame_id = "world"
                leg.id = f"table_leg_{index}"

                leg_primitive = SolidPrimitive()

                leg_primitive.type = (
                    SolidPrimitive.BOX
                )

                leg_primitive.dimensions = [
                    0.05,
                    0.05,
                    0.45
                ]

                leg_pose = Pose()

                leg_pose.position.x = x
                leg_pose.position.y = y
                leg_pose.position.z = 0.225

                leg_pose.orientation.w = 1.0

                leg.primitives.append(
                    leg_primitive
                )

                leg.primitive_poses.append(
                    leg_pose
                )

                leg.operation = (
                    CollisionObject.ADD
                )

                scene.apply_collision_object(
                    leg
                )

            # Force planning scene state update
            scene.current_state.update()

        self.node.get_logger().info(
            "Table collision geometry added to MoveIt"
        )

    # ========================================================
    # MOVE TO CARTESIAN POSE
    # ========================================================

    def move_to_pose(
        self,
        x,
        y,
        z
    ):

        pose = PoseStamped()

        pose.header.frame_id = "world"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        pose.pose.orientation.x = QX
        pose.pose.orientation.y = QY
        pose.pose.orientation.z = QZ
        pose.pose.orientation.w = QW

        for attempt in range(2):

            time.sleep(0.5)

            self.arm.set_start_state_to_current_state()

            self.arm.set_goal_state(
                pose_stamped_msg=pose,
                pose_link="tool0"
            )

            plan = self.arm.plan()

            if not plan:

                self.node.get_logger().warning(
                    f"Planning failed for "
                    f"({x:.3f}, {y:.3f}, {z:.3f})"
                )

                continue

            self.node.get_logger().info(
                f"Moving to "
                f"({x:.3f}, {y:.3f}, {z:.3f})"
            )

            self.robot.execute(
                plan.trajectory,
                controllers=[
                    ARM_CONTROLLER
                ]
            )

            time.sleep(1.0)

            return True

        return False

    # ========================================================
    # HOME
    # ========================================================

    def home(self):

        time.sleep(0.5)

        self.arm.set_start_state_to_current_state()

        self.arm.set_goal_state(
            configuration_name="home"
        )

        plan = self.arm.plan()

        if not plan:

            self.node.get_logger().error(
                "Could not plan home"
            )

            return False

        self.robot.execute(
            plan.trajectory,
            controllers=[
                ARM_CONTROLLER
            ]
        )

        time.sleep(1.0)

        return True

    # ========================================================
    # GRIPPER COMMAND
    # ========================================================

    def gripper_command(
        self,
        position
    ):

        if not self.gripper.wait_for_server(
            timeout_sec=10.0
        ):
            self.node.get_logger().error(
                "Gripper action server unavailable"
            )
            return False

        goal = ParallelGripperCommand.Goal()

        goal.command.name = [
            GRIPPER_JOINT
        ]

        goal.command.position = [
            position
        ]

        goal.command.effort = [
            GRIPPER_EFFORT
        ]

        future = self.gripper.send_goal_async(
            goal
        )

        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=10.0
        )

        if future.result() is None:
            self.node.get_logger().error(
                "No gripper goal response"
            )
            return False

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.node.get_logger().error(
                "Gripper goal rejected"
            )
            return False

        self.node.get_logger().info(
            "Gripper goal accepted"
        )

        # Do not wait for the simulated action result.
        # The goal has been accepted by the controller;
        # allow Gazebo time to physically execute it.
        time.sleep(1.0)

        return True

    # ========================================================
    # OPEN
    # ========================================================

    def open_gripper(self):

        self.node.get_logger().info(
            "Opening gripper"
        )

        return self.gripper_command(
            GRIPPER_OPEN
        )

    # ========================================================
    # CLOSE
    # ========================================================

    def close_gripper(self):

        self.node.get_logger().info(
            "Closing gripper"
        )

        return self.gripper_command(
            GRIPPER_CLOSED
        )




# ============================================================
# MAIN
# ============================================================

def main():

    rclpy.init()

    perception = Perception()

    controller = PickPlace(
        perception
    )

    try:

        time.sleep(2.0)

        perception.get_logger().info(
            "Starting autonomous pick-and-place"
        )

        # ----------------------------------------------------
        # Open gripper
        # ----------------------------------------------------

        if not controller.open_gripper():
            return

        # ----------------------------------------------------
        # Home
        # ----------------------------------------------------

        if not controller.home():

            perception.get_logger().error(
                "Could not reach home"
            )

            return

        picked = 0

        # ----------------------------------------------------
        # Scan
        # ----------------------------------------------------

        for cycle in range(
            MAX_SCAN_CYCLES
        ):

            if picked >= EXPECTED_BOXES:
                break

            perception.get_logger().info(
                f"===== SCAN CYCLE {cycle + 1} ====="
            )

            for scan_number, (
                scan_x,
                scan_y
            ) in enumerate(
                SCAN_POSITIONS,
                start=1
            ):

                # Stop immediately once all four objects
                # have been successfully picked.
                if picked >= EXPECTED_BOXES:
                    break

                perception.get_logger().info(
                    f"===== SCAN {scan_number} ====="
                )

                perception.get_logger().info(
                    f"Scan pose: "
                    f"({scan_x:.2f}, "
                    f"{scan_y:.2f})"
                )

                # ----------------------------------------------------
                # Move camera to scan pose
                # ----------------------------------------------------

                if not controller.move_to_pose(
                    scan_x,
                    scan_y,
                    PRE_GRASP_Z
                ):
                    continue

                # ----------------------------------------------------
                # Detect objects
                # ----------------------------------------------------

                boxes = perception.collect_boxes(
                    duration_sec=2.5
                )

                if not boxes:

                    perception.get_logger().info(
                        "No reliable boxes detected"
                    )

                    continue

                perception.get_logger().info(
                    f"Detected "
                    f"{len(boxes)} candidate boxes"
                )

                # Closest object to current scan position
                boxes.sort(
                    key=lambda box:
                    (
                        box["x"] - scan_x
                    ) ** 2
                    +
                    (
                        box["y"] - scan_y
                    ) ** 2
                )

                box = boxes[0]

                color = box["color"]
                box_x = box["x"]
                box_y = box["y"]

                perception.get_logger().info(
                    f"Selected {color} box at "
                    f"({box_x:.3f}, "
                    f"{box_y:.3f})"
                )

                # ----------------------------------------------------
                # Move directly above object
                # ----------------------------------------------------

                if not controller.move_to_pose(
                    box_x,
                    box_y,
                    PRE_GRASP_Z
                ):
                    continue

                # ----------------------------------------------------
                # Open
                # ----------------------------------------------------

                if not controller.open_gripper():
                    return

                # ----------------------------------------------------
                # Descend
                # ----------------------------------------------------

                perception.get_logger().info(
                    f"Descending to grasp pose: "
                    f"({box_x:.3f}, {box_y:.3f}, {GRASP_Z:.3f})"
                )

                if not controller.move_to_pose(
                    box_x,
                    box_y,
                    GRASP_Z
                ):
                    perception.get_logger().warning(
                        "Could not reach grasp pose"
                    )
                    continue

                # Give the robot a moment to settle
                time.sleep(0.5)

                # ----------------------------------------------------
                # Close gripper
                # ----------------------------------------------------

                if not controller.close_gripper():
                    perception.get_logger().error(
                        "Could not command gripper to close"
                    )
                    return

                perception.get_logger().info(
                    "Gripper close command accepted"
                )

                # ----------------------------------------------------
                # Lift
                # ----------------------------------------------------

                if not controller.move_to_pose(
                    box_x,
                    box_y,
                    LIFT_Z
                ):

                    perception.get_logger().error(
                        "Could not lift object"
                    )

                    controller.open_gripper()

                    continue

                # ----------------------------------------------------
                # Select bin
                # ----------------------------------------------------

                if color == "red":

                    bin_x, bin_y = RED_BIN

                else:

                    bin_x, bin_y = BLUE_BIN

                perception.get_logger().info(
                    f"Moving {color} object to bin "
                    f"({bin_x:.2f}, "
                    f"{bin_y:.2f})"
                )

                time.sleep(1.0)

                # ----------------------------------------------------
                # Move to bin
                # ----------------------------------------------------

                if not controller.move_to_pose(
                    bin_x,
                    bin_y,
                    LIFT_Z
                ):

                    perception.get_logger().error(
                        "Could not reach bin"
                    )

                    continue

                # ----------------------------------------------------
                # Release
                # ----------------------------------------------------

                if not controller.open_gripper():
                    return

                time.sleep(1.0)

                # The placement sequence completed.
                picked += 1

                perception.get_logger().info(
                    f"Placed {color} object."
                )

                perception.get_logger().info(
                    f"Objects placed: "
                    f"{picked}/{EXPECTED_BOXES}"
                )

                # ----------------------------------------------------
                # Return home
                # ----------------------------------------------------

                if picked < EXPECTED_BOXES:

                    if not controller.home():
                        return

        perception.get_logger().info(
            "================================"
        )

        perception.get_logger().info(
            "PICK AND PLACE COMPLETE"
        )

        perception.get_logger().info(
            f"Total boxes picked: {picked}"
        )

        perception.get_logger().info(
            "================================"
        )

    except KeyboardInterrupt:

        perception.get_logger().info(
            "Stopped by user"
        )

    except Exception as exc:

        perception.get_logger().error(
            f"Exception: {exc}"
        )

    finally:

        if rclpy.ok():

            perception.destroy_node()

            rclpy.shutdown()


if __name__ == "__main__":

    main()