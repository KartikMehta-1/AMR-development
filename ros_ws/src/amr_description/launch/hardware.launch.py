from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_dev = LaunchConfiguration("agent_dev")
    agent_baud = LaunchConfiguration("agent_baud")
    start_lidar = LaunchConfiguration("start_lidar")
    start_camera = LaunchConfiguration("start_camera")
    start_link_watchdog = LaunchConfiguration("start_link_watchdog")
    lidar_params = LaunchConfiguration("lidar_params")

    model_path = PathJoinSubstitution(
        [FindPackageShare("amr_description"), "urdf", "amr.urdf.xacro"]
    )
    robot_description = Command([
        "xacro", " ", model_path,
        " ", "use_sim:=false",
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": use_sim_time},
        ],
    )

    amr_link_watchdog = Node(
        package="amr_hardware",
        executable="amr_link_watchdog",
        output="screen",
        condition=IfCondition(start_link_watchdog),
        parameters=[
            {
                "wheel_state_topic": "/amr_stm/wheel_state",
                "startup_timeout_sec": 5.0,
                "stale_timeout_sec": 1.0,
                "publish_period_sec": 0.5,
            }
        ],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            PathJoinSubstitution(
                [FindPackageShare("amr_description"), "config", "ros2_control.yaml"]
            ),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner.py",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ],
    )

    diff_drive_controller_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner.py",
                arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ],
    )

    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ydlidar_ros2_driver"), "launch", "ydlidar_launch.py"]
            )
        ),
        launch_arguments={"params_file": lidar_params}.items(),
        condition=IfCondition(start_lidar),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "agent_dev",
                default_value="/dev/ttyACM0",
                description="Serial device for micro-ROS agent",
            ),
            DeclareLaunchArgument(
                "agent_baud",
                default_value="460800",
                description="Baud rate for micro-ROS agent",
            ),
            DeclareLaunchArgument(
                "start_lidar",
                default_value="true",
                description="Start YDLidar driver",
            ),
            DeclareLaunchArgument(
                "start_camera",
                default_value="false",
                description="Start RealSense camera driver",
            ),
            DeclareLaunchArgument(
                "start_link_watchdog",
                default_value="true",
                description="Publish STM link health from /amr_stm/wheel_state freshness",
            ),
            DeclareLaunchArgument(
                "lidar_params",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("amr_description"), "config", "ydlidar.yaml"]
                ),
                description="YDLidar params file path",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "micro_ros_agent",
                    "micro_ros_agent",
                    "serial",
                    "--dev",
                    agent_dev,
                    "-b",
                    agent_baud,
                ],
                respawn=True,
                respawn_delay=2.0,
                output="screen",
            ),
            SetEnvironmentVariable("RCUTILS_LOGGING_SEVERITY_THRESHOLD", "ERROR"),
            ydlidar_launch,
            SetEnvironmentVariable("RCUTILS_LOGGING_SEVERITY_THRESHOLD", "INFO"),
            ExecuteProcess(
                condition=IfCondition(start_camera),
                cmd=[
                    "ros2",
                    "launch",
                    "realsense2_camera",
                    "rs_launch.py",
                    "pointcloud.enable:=true",
                    "align_depth.enable:=true",
                ],
                output="screen",
            ),
            robot_state_publisher,
            amr_link_watchdog,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            diff_drive_controller_spawner,
        ]
    )
