from __future__ import annotations

import copy
from typing import Optional

import cv2
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class ImageRotate180(Node):
    """Publish rotated/resized image streams for display and datasets."""

    def __init__(self) -> None:
        super().__init__("image_rotate_180")

        self.declare_parameter("input_image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("input_camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("output_image_topic", "/camera/camera/color/image_raw_rotated")
        self.declare_parameter(
            "output_camera_info_topic", "/camera/camera/color/camera_info_rotated"
        )
        self.declare_parameter("output_frame_id", "")
        self.declare_parameter("rotate_180", True)
        self.declare_parameter("output_width", 0)
        self.declare_parameter("output_height", 0)
        self.declare_parameter("max_rate_hz", 0.0)

        self._bridge = CvBridge()
        self._latest_camera_info: Optional[CameraInfo] = None
        self._output_frame_id = (
            self.get_parameter("output_frame_id").get_parameter_value().string_value
        )
        self._rotate_180 = self.get_parameter("rotate_180").get_parameter_value().bool_value
        self._output_width = (
            self.get_parameter("output_width").get_parameter_value().integer_value
        )
        self._output_height = (
            self.get_parameter("output_height").get_parameter_value().integer_value
        )
        self._max_rate_hz = (
            self.get_parameter("max_rate_hz").get_parameter_value().double_value
        )
        self._last_publish_ns = 0

        input_image_topic = (
            self.get_parameter("input_image_topic").get_parameter_value().string_value
        )
        input_camera_info_topic = (
            self.get_parameter("input_camera_info_topic").get_parameter_value().string_value
        )
        output_image_topic = (
            self.get_parameter("output_image_topic").get_parameter_value().string_value
        )
        output_camera_info_topic = (
            self.get_parameter("output_camera_info_topic").get_parameter_value().string_value
        )

        self._image_pub = self.create_publisher(
            Image, output_image_topic, qos_profile_sensor_data
        )
        self._camera_info_pub = self.create_publisher(CameraInfo, output_camera_info_topic, 10)
        self.create_subscription(
            CameraInfo, input_camera_info_topic, self._camera_info_callback, 10
        )
        self.create_subscription(Image, input_image_topic, self._image_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f"Transforming {input_image_topic} -> {output_image_topic}; "
            f"rotate_180={self._rotate_180}, "
            f"output={self._output_width}x{self._output_height}, "
            f"max_rate_hz={self._max_rate_hz}"
        )

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self._latest_camera_info = msg

    def _image_callback(self, msg: Image) -> None:
        if self._max_rate_hz > 0.0:
            now_ns = self.get_clock().now().nanoseconds
            min_period_ns = int(1e9 / self._max_rate_hz)
            if self._last_publish_ns and now_ns - self._last_publish_ns < min_period_ns:
                return
            self._last_publish_ns = now_ns

        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            transformed = cv2.rotate(image, cv2.ROTATE_180) if self._rotate_180 else image
            if self._output_width > 0 and self._output_height > 0:
                interpolation = cv2.INTER_NEAREST if msg.encoding in ("16UC1", "mono16") else cv2.INTER_AREA
                transformed = cv2.resize(
                    transformed,
                    (self._output_width, self._output_height),
                    interpolation=interpolation,
                )
            out_msg = self._bridge.cv2_to_imgmsg(transformed, encoding=msg.encoding)
        except CvBridgeError as exc:
            self.get_logger().warning(f"Failed to transform image: {exc}")
            return

        out_msg.header = copy.deepcopy(msg.header)
        if self._output_frame_id:
            out_msg.header.frame_id = self._output_frame_id
        self._image_pub.publish(out_msg)

        if self._latest_camera_info is not None:
            info_msg = self._transformed_camera_info(self._latest_camera_info)
            info_msg.header.stamp = msg.header.stamp
            if self._output_frame_id:
                info_msg.header.frame_id = self._output_frame_id
            self._camera_info_pub.publish(info_msg)

    def _transformed_camera_info(self, msg: CameraInfo) -> CameraInfo:
        transformed = copy.deepcopy(msg)
        input_width = transformed.width
        input_height = transformed.height

        if self._rotate_180 and input_width > 0 and input_height > 0:
            transformed.k[2] = float(input_width - 1) - transformed.k[2]
            transformed.k[5] = float(input_height - 1) - transformed.k[5]
            transformed.p[2] = float(input_width - 1) - transformed.p[2]
            transformed.p[6] = float(input_height - 1) - transformed.p[6]

        if (
            self._output_width > 0
            and self._output_height > 0
            and input_width > 0
            and input_height > 0
        ):
            scale_x = float(self._output_width) / float(input_width)
            scale_y = float(self._output_height) / float(input_height)
            transformed.width = self._output_width
            transformed.height = self._output_height
            transformed.k[0] *= scale_x
            transformed.k[2] = ((transformed.k[2] + 0.5) * scale_x) - 0.5
            transformed.k[4] *= scale_y
            transformed.k[5] = ((transformed.k[5] + 0.5) * scale_y) - 0.5
            transformed.p[0] *= scale_x
            transformed.p[2] = ((transformed.p[2] + 0.5) * scale_x) - 0.5
            transformed.p[5] *= scale_y
            transformed.p[6] = ((transformed.p[6] + 0.5) * scale_y) - 0.5

        return transformed


def main(args: list[str] | None = None) -> None:
    import rclpy

    rclpy.init(args=args)
    node = ImageRotate180()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
