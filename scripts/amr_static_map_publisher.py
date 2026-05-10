#!/usr/bin/env python3

import argparse
from pathlib import Path

import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def read_pgm(path: Path):
    def token(handle):
        data = b""
        while True:
            char = handle.read(1)
            if not char:
                return data.decode("ascii")
            if char == b"#":
                handle.readline()
                continue
            if char.isspace():
                if data:
                    return data.decode("ascii")
                continue
            data += char

    with path.open("rb") as handle:
        magic = token(handle)
        width = int(token(handle))
        height = int(token(handle))
        maxval = int(token(handle))
        if magic == "P5":
            pixels = list(handle.read(width * height))
        elif magic == "P2":
            pixels = [int(token(handle)) for _ in range(width * height)]
        else:
            raise RuntimeError(f"unsupported PGM format: {magic}")
    return width, height, maxval, pixels


def load_map(path: Path):
    metadata = yaml.safe_load(path.read_text(encoding="utf-8"))
    image_path = Path(metadata["image"])
    if not image_path.is_absolute():
        image_path = path.parent / image_path

    width, height, maxval, pixels = read_pgm(image_path)
    negate = int(metadata.get("negate", 0))
    occupied_thresh = float(metadata.get("occupied_thresh", 0.65))
    free_thresh = float(metadata.get("free_thresh", 0.25))

    data = []
    for row in range(height - 1, -1, -1):
        for col in range(width):
            value = pixels[row * width + col] / float(maxval)
            occupancy = value if negate else 1.0 - value
            if occupancy > occupied_thresh:
                data.append(100)
            elif occupancy < free_thresh:
                data.append(0)
            else:
                data.append(-1)

    return metadata, image_path, width, height, data


def parse_args():
    parser = argparse.ArgumentParser(description="Publish a static map on /map with transient-local QoS.")
    parser.add_argument("map_yaml", help="Map YAML path.")
    parser.add_argument("--topic", default="/map")
    parser.add_argument("--period", type=float, default=1.0)
    return parser.parse_args()


def main():
    args = parse_args()
    map_yaml = Path(args.map_yaml)
    metadata, image_path, width, height, data = load_map(map_yaml)
    origin = metadata.get("origin", [0.0, 0.0, 0.0])

    rclpy.init()
    node = Node("amr_static_map_publisher")
    qos = QoSProfile(depth=1)
    qos.history = HistoryPolicy.KEEP_LAST
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    publisher = node.create_publisher(OccupancyGrid, args.topic, qos)

    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.resolution = float(metadata["resolution"])
    message.info.width = width
    message.info.height = height
    message.info.origin.position.x = float(origin[0])
    message.info.origin.position.y = float(origin[1])
    message.info.origin.position.z = 0.0
    message.info.origin.orientation.w = 1.0
    message.data = data

    node.get_logger().info(
        f"publishing {image_path} on {args.topic}: "
        f"{width}x{height} res={message.info.resolution} origin={origin[:2]}"
    )

    try:
        while rclpy.ok():
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=max(0.1, args.period))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
