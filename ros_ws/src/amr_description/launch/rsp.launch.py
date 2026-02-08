from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim = LaunchConfiguration("use_sim")

    model_path = PathJoinSubstitution(
        [FindPackageShare("amr_description"), "urdf", "amr.urdf.xacro"]
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
                description="Generate URDF for sim if true",
            ),
            robot_state_publisher,
        ]
    )
