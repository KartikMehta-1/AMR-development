from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_params = LaunchConfiguration("camera_params")
    video_device = LaunchConfiguration("video_device")
    start_preview = LaunchConfiguration("start_preview")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_params",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("amr_description"),
                        "config",
                        "so101_wrist_webcam.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("start_preview", default_value="true"),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="so101_wrist_camera",
                namespace="so101",
                output="screen",
                parameters=[
                    camera_params,
                    {
                        "video_device": video_device,
                        "camera_frame_id": "so101_wrist_camera_optical_frame",
                    },
                ],
                remappings=[
                    ("image_raw", "wrist_camera/image_raw"),
                    ("camera_info", "wrist_camera/camera_info"),
                ],
            ),
            Node(
                package="amr_perception",
                executable="image_rotate_180",
                name="so101_wrist_camera_preview",
                output="screen",
                condition=IfCondition(start_preview),
                parameters=[
                    {
                        "input_image_topic": "/so101/wrist_camera/image_raw",
                        "input_camera_info_topic": "/so101/wrist_camera/camera_info",
                        "output_image_topic": "/so101/preview/wrist_camera/image_raw",
                        "output_camera_info_topic": "/so101/preview/wrist_camera/camera_info",
                        "rotate_180": False,
                        "output_width": 320,
                        "output_height": 240,
                        "max_rate_hz": 10.0,
                    }
                ],
            ),
        ]
    )
