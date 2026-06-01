from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware_launch = PathJoinSubstitution(
        [FindPackageShare("amr_description"), "launch", "hardware.launch.py"]
    )
    lidar_params = PathJoinSubstitution(
        [FindPackageShare("amr_description"), "config", "ydlidar_orin.yaml"]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_lidar = LaunchConfiguration("start_lidar")
    start_camera = LaunchConfiguration("start_camera")
    start_link_watchdog = LaunchConfiguration("start_link_watchdog")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_camera", default_value="false"),
            DeclareLaunchArgument("start_link_watchdog", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(hardware_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "agent_dev": (
                        "/dev/serial/by-id/"
                        "usb-STMicroelectronics_STM32_STLink_066CFF34314B4E3043064322-if02"
                    ),
                    "agent_baud": "460800",
                    "start_lidar": start_lidar,
                    "start_camera": start_camera,
                    "start_link_watchdog": start_link_watchdog,
                    "lidar_params": lidar_params,
                }.items(),
            ),
        ]
    )
