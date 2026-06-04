from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_params = LaunchConfiguration("camera_params")
    video_device = LaunchConfiguration("video_device")

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
        ]
    )
