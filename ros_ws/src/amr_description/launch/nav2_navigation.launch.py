import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    map_subscribe_transient_local = LaunchConfiguration("map_subscribe_transient_local")

    # Prefer paths relative to this file so it works both:
    # - directly from source (ros2 launch /path/to/nav2_navigation.launch.py)
    # - from an installed package (symlink-install or normal install)
    _pkg_root = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))
    default_nav2_params = os.path.join(_pkg_root, "config", "nav2_params_amr.yaml")

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": params_file,
            "map_subscribe_transient_local": map_subscribe_transient_local,
        }.items(),
    )

    nav2_group = GroupAction(
        [
            # Wire Nav2 defaults (/cmd_vel, /odom) to ros2_control diff_drive topics.
            SetRemap(src="cmd_vel", dst=cmd_vel_topic),
            SetRemap(src="/cmd_vel", dst=cmd_vel_topic),
            SetRemap(src="odom", dst=odom_topic),
            SetRemap(src="/odom", dst=odom_topic),
            nav2_navigation,
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_nav2_params,
                description="Nav2 params YAML",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically startup the nav2 stack",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/diff_drive_controller/cmd_vel_unstamped",
                description="Where Nav2 should publish base velocity commands (Twist)",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/diff_drive_controller/odom",
                description="Where Nav2 should read odometry (nav_msgs/Odometry)",
            ),
            DeclareLaunchArgument(
                "map_subscribe_transient_local",
                default_value="true",
                description="Subscribe to /map using transient_local QoS",
            ),
            nav2_group,
        ]
    )

