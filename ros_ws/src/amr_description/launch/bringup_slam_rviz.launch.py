from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim = LaunchConfiguration("use_sim")
    use_rviz = LaunchConfiguration("use_rviz")
    slam_params = LaunchConfiguration("slam_params_file")
    rviz_config = LaunchConfiguration("rviz_config")
    slam_delay = LaunchConfiguration("slam_delay")
    rviz_delay = LaunchConfiguration("rviz_delay")

    model_path = PathJoinSubstitution(
        [FindPackageShare("amr_description"), "urdf", "amr.urdf.xacro"]
    )
    default_slam_params = PathJoinSubstitution(
        [FindPackageShare("amr_description"), "config", "slam_toolbox_online_async.yaml"]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("amr_description"), "config", "amr.rviz"]
    )

    robot_description = Command(
        [
            "xacro",
            " ",
            model_path,
            " ",
            "use_sim:=",
            use_sim,
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": use_sim_time},
        ],
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": use_sim_time}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    delayed_slam_toolbox = TimerAction(
        period=slam_delay,
        actions=[slam_toolbox],
    )

    delayed_rviz = TimerAction(
        period=rviz_delay,
        actions=[rviz],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="AMR_DESCRIPTION_SHARE",
                value=FindPackageShare("amr_description"),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "use_sim",
                default_value="false",
                description="Generate URDF for simulation if true",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz2 if true",
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=default_slam_params,
                description="SLAM Toolbox YAML config file",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz config file path",
            ),
            DeclareLaunchArgument(
                "slam_delay",
                default_value="2.0",
                description="Delay before starting slam_toolbox (seconds)",
            ),
            DeclareLaunchArgument(
                "rviz_delay",
                default_value="4.0",
                description="Delay before starting RViz2 (seconds)",
            ),
            robot_state_publisher,
            delayed_slam_toolbox,
            delayed_rviz,
        ]
    )
