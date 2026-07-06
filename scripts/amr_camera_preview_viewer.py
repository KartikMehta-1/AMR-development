#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class PreviewViewer(Node):
    def __init__(self):
        super().__init__("amr_camera_preview_viewer")
        self.bridge = CvBridge()
        self.frames = {}
        self.create_subscription(
            Image,
            "/camera/preview/color/image_raw",
            lambda msg: self._store_color("RealSense color preview", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/camera/preview/depth/image_rect_raw",
            lambda msg: self._store_depth("RealSense depth preview", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/so101/preview/wrist_camera/image_raw",
            lambda msg: self._store_color("SO-101 wrist preview", msg),
            qos_profile_sensor_data,
        )
        self.create_timer(0.03, self._show)
        self.get_logger().info("Preview windows opening. Press q or Esc in a window to exit.")

    def _store_color(self, name, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.frames[name] = frame

    def _store_depth(self, name, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        finite = depth[np.isfinite(depth)] if np.issubdtype(depth.dtype, np.floating) else depth
        valid = finite[finite > 0]
        if valid.size:
            near = np.percentile(valid, 2)
            far = np.percentile(valid, 98)
            if far <= near:
                far = near + 1.0
            scaled = np.clip((depth.astype(np.float32) - near) * 255.0 / (far - near), 0, 255)
        else:
            scaled = np.zeros(depth.shape, dtype=np.float32)
        self.frames[name] = cv2.applyColorMap(scaled.astype(np.uint8), cv2.COLORMAP_TURBO)

    def _show(self):
        for name, frame in self.frames.items():
            cv2.imshow(name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            rclpy.shutdown()


def main():
    rclpy.init()
    node = PreviewViewer()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()


if __name__ == "__main__":
    main()
