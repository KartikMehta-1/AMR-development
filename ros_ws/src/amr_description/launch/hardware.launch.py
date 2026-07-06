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
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_dev = LaunchConfiguration("agent_dev")
    agent_baud = LaunchConfiguration("agent_baud")
    start_lidar = LaunchConfiguration("start_lidar")
    start_camera = LaunchConfiguration("start_camera")
    start_link_watchdog = LaunchConfiguration("start_link_watchdog")
    lidar_params = LaunchConfiguration("lidar_params")
    joint_states_topic = LaunchConfiguration("joint_states_topic")

    model_path = PathJoinSubstitution(
        [FindPackageShare("amr_description"), "urdf", "amr.urdf.xacro"]
    )
    robot_description = ParameterValue(
        Command([
            "xacro", " ", model_path,
            " ", "use_sim:=false",
        ]),
        value_type=str,
    )

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
                "stale_timeout_sec": 2.0,
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
        remappings=[("/joint_states", joint_states_topic)],
        output="screen",
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
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
                executable="spawner",
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

    realsense_color_preview = Node(
        package="amr_perception",
        executable="image_rotate_180",
        name="realsense_color_preview",
        output="screen",
        condition=IfCondition(start_camera),
        parameters=[
            {
                "input_image_topic": "/camera/camera/color/image_raw",
                "input_camera_info_topic": "/camera/camera/color/camera_info",
                "output_image_topic": "/camera/preview/color/image_raw",
                "output_camera_info_topic": "/camera/preview/color/camera_info",
                "rotate_180": True,
                "output_width": 320,
                "output_height": 240,
                "max_rate_hz": 6.0,
            }
        ],
    )

    realsense_depth_preview = Node(
        package="amr_perception",
        executable="image_rotate_180",
        name="realsense_depth_preview",
        output="screen",
        condition=IfCondition(start_camera),
        parameters=[
            {
                "input_image_topic": "/camera/camera/depth/image_rect_raw",
                "input_camera_info_topic": "/camera/camera/depth/camera_info",
                "output_image_topic": "/camera/preview/depth/image_rect_raw",
                "output_camera_info_topic": "/camera/preview/depth/camera_info",
                "rotate_180": True,
                "output_width": 320,
                "output_height": 240,
                "max_rate_hz": 6.0,
            }
        ],
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
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="/joint_states",
                description=(
                    "JointState topic for AMR base joints; use /amr/joint_states "
                    "when merging with SO-101"
                ),
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
                    "rgb_camera.color_profile:=640x480x15",
                    "depth_module.depth_profile:=640x480x15",
                    "pointcloud.enable:=false",
                    "align_depth.enable:=false",
                    "spatial_filter.enable:=false",
                    "temporal_filter.enable:=false",
                    "hole_filling_filter.enable:=false",
                    "colorizer.enable:=false",
                ],
                output="screen",
            ),
            realsense_color_preview,
            realsense_depth_preview,
            robot_state_publisher,
            amr_link_watchdog,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            diff_drive_controller_spawner,
        ]
    )
