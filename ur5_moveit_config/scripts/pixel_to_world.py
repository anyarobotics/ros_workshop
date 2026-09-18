#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener, TransformException

import cv2


TABLE_Z = 0.50

OPTICAL_FRAME = "wrist_camera_optical_link"


class PixelToWorld(Node):

    def __init__(self):
        super().__init__("pixel_to_world")

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.latest_image = None

        # --------------------------------------------------------
        # TF
        # --------------------------------------------------------

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # --------------------------------------------------------
        # Camera info
        # --------------------------------------------------------

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/wrist_camera/camera_info",
            self.camera_info_callback,
            10,
        )

        # --------------------------------------------------------
        # Image
        # --------------------------------------------------------

        self.image_sub = self.create_subscription(
            Image,
            "/wrist_camera/image_raw",
            self.image_callback,
            10,
        )

        self.get_logger().info(
            "Pixel-to-world node started"
        )

    # ============================================================
    # Camera info
    # ============================================================

    def camera_info_callback(self, msg):

        self.fx = msg.k[0]
        self.fy = msg.k[4]

        self.cx = msg.k[2]
        self.cy = msg.k[5]

    # ============================================================
    # Image
    # ============================================================

    def image_callback(self, msg):

        if self.fx is None:
            return

        if msg.encoding not in ("rgb8", "bgr8"):
            return

        image = np.frombuffer(
            msg.data,
            dtype=np.uint8,
        ).reshape(
            msg.height,
            msg.width,
            3,
        )

        if msg.encoding == "rgb8":

            image = cv2.cvtColor(
                image,
                cv2.COLOR_RGB2BGR,
            )

        hsv = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2HSV,
        )

        red_mask = self.get_red_mask(hsv)
        blue_mask = self.get_blue_mask(hsv)

        self.process_color(
            red_mask,
            "RED",
            msg,
        )

        self.process_color(
            blue_mask,
            "BLUE",
            msg,
        )

    # ============================================================
    # Color masks
    # ============================================================

    def get_red_mask(self, hsv):

        lower1 = np.array(
            [0, 120, 80]
        )

        upper1 = np.array(
            [10, 255, 255]
        )

        lower2 = np.array(
            [170, 120, 80]
        )

        upper2 = np.array(
            [180, 255, 255]
        )

        mask1 = cv2.inRange(
            hsv,
            lower1,
            upper1,
        )

        mask2 = cv2.inRange(
            hsv,
            lower2,
            upper2,
        )

        return mask1 | mask2

    def get_blue_mask(self, hsv):

        lower = np.array(
            [90, 100, 50]
        )

        upper = np.array(
            [140, 255, 255]
        )

        return cv2.inRange(
            hsv,
            lower,
            upper,
        )

    # ============================================================
    # Detect centroid
    # ============================================================

    def process_color(
        self,
        mask,
        name,
        image_msg,
    ):

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        if not contours:
            return

        contour = max(
            contours,
            key=cv2.contourArea,
        )

        area = cv2.contourArea(contour)

        if area < 20:
            return

        moments = cv2.moments(contour)

        if moments["m00"] == 0:
            return

        u = (
            moments["m10"]
            / moments["m00"]
        )

        v = (
            moments["m01"]
            / moments["m00"]
        )

        world_point = self.pixel_to_world(
            u,
            v,
            image_msg,
        )

        if world_point is None:
            return

        x, y, z = world_point

        self.get_logger().info(
            f"{name}: "
            f"pixel=({u:.1f}, {v:.1f}) "
            f"world=({x:.3f}, {y:.3f}, {z:.3f})"
        )

    # ============================================================
    # Pixel -> world
    # ============================================================

    def pixel_to_world(
        self,
        u,
        v,
        image_msg,
    ):

        # --------------------------------------------------------
        # Pixel -> camera optical ray
        # --------------------------------------------------------

        ray_camera = np.array(
            [
                (u - self.cx) / self.fx,
                (v - self.cy) / self.fy,
                1.0,
            ],
            dtype=float,
        )

        # --------------------------------------------------------
        # Get camera -> world transform
        # --------------------------------------------------------

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    "world",
                    OPTICAL_FRAME,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(
                        seconds=0.2
                    ),
                )
            )

        except TransformException as ex:

            self.get_logger().warn(
                f"TF lookup failed: {ex}"
            )

            return None

        # --------------------------------------------------------
        # Translation
        # --------------------------------------------------------

        t = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ]
        )

        # --------------------------------------------------------
        # Quaternion -> rotation matrix
        # --------------------------------------------------------

        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        R = np.array(
            [
                [
                    1 - 2 * (qy*qy + qz*qz),
                    2 * (qx*qy - qz*qw),
                    2 * (qx*qz + qy*qw),
                ],

                [
                    2 * (qx*qy + qz*qw),
                    1 - 2 * (qx*qx + qz*qz),
                    2 * (qy*qz - qx*qw),
                ],

                [
                    2 * (qx*qz - qy*qw),
                    2 * (qy*qz + qx*qw),
                    1 - 2 * (qx*qx + qy*qy),
                ],
            ]
        )

        # --------------------------------------------------------
        # Ray in world frame
        # --------------------------------------------------------

        ray_world = R @ ray_camera

        # --------------------------------------------------------
        # Intersect ray with table plane:
        #
        #       z = TABLE_Z
        # --------------------------------------------------------

        if abs(ray_world[2]) < 1e-9:

            return None

        distance = (
            TABLE_Z - t[2]
        ) / ray_world[2]

        if distance <= 0:

            return None

        point_world = (
            t
            + distance * ray_world
        )

        return point_world


def main(args=None):

    rclpy.init(args=args)

    node = PixelToWorld()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        


if __name__ == "__main__":

    main()
