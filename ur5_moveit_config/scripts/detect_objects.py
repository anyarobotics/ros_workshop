#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import cv2

from sensor_msgs.msg import Image


class ObjectDetector(Node):

    def __init__(self):
        super().__init__("object_detector")

        self.subscription = self.create_subscription(
            Image,
            "/wrist_camera/image_raw",
            self.image_callback,
            10,
        )

        self.get_logger().info("Object detector started")

    def image_callback(self, msg):

        # --------------------------------------------------------
        # Convert ROS Image -> NumPy image
        # --------------------------------------------------------

        if msg.encoding not in ("rgb8", "bgr8"):
            self.get_logger().warn(
                f"Unsupported encoding: {msg.encoding}"
            )
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

        # --------------------------------------------------------
        # BGR -> HSV
        # --------------------------------------------------------

        hsv = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2HSV,
        )

        # --------------------------------------------------------
        # RED
        # --------------------------------------------------------

        red_lower_1 = np.array([0, 120, 80])
        red_upper_1 = np.array([10, 255, 255])

        red_lower_2 = np.array([170, 120, 80])
        red_upper_2 = np.array([180, 255, 255])

        red_mask_1 = cv2.inRange(
            hsv,
            red_lower_1,
            red_upper_1,
        )

        red_mask_2 = cv2.inRange(
            hsv,
            red_lower_2,
            red_upper_2,
        )

        red_mask = red_mask_1 | red_mask_2

        # --------------------------------------------------------
        # BLUE
        # --------------------------------------------------------

        blue_lower = np.array([90, 100, 50])
        blue_upper = np.array([140, 255, 255])

        blue_mask = cv2.inRange(
            hsv,
            blue_lower,
            blue_upper,
        )

        # --------------------------------------------------------
        # Find largest contour
        # --------------------------------------------------------

        self.detect_color(
            red_mask,
            "RED",
        )

        self.detect_color(
            blue_mask,
            "BLUE",
        )

    def detect_color(self, mask, name):

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        if not contours:
            return

        largest = max(
            contours,
            key=cv2.contourArea,
        )

        area = cv2.contourArea(largest)

        # Ignore tiny noise
        if area < 20:
            return

        moments = cv2.moments(largest)

        if moments["m00"] == 0:
            return

        u = int(
            moments["m10"] /
            moments["m00"]
        )

        v = int(
            moments["m01"] /
            moments["m00"]
        )

        self.get_logger().info(
            f"{name}: pixel=({u}, {v}), area={area:.1f}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = ObjectDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
