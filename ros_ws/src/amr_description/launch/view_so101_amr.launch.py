import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_TEMP_URDF_FILES = []


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration("use_rviz")
    use_joint_state_gui = LaunchConfiguration("use_joint_state_gui").perform(context)
    package_share = get_package_share_directory("amr_description")
    model_path = os.path.join(package_share, "urdf", "amr.urdf.xacro")
    rviz_config_path = os.path.join(package_share, "config", "so101_amr.rviz")

    mappings = {
        "enable_so101": "true",
        "so101_mount_x": LaunchConfiguration("mount_x").perform(context),
        "so101_mount_y": LaunchConfiguration("mount_y").perform(context),
        "so101_mount_z": LaunchConfiguration("mount_z").perform(context),
        "so101_mount_roll": LaunchConfiguration("mount_roll").perform(context),
        "so101_mount_pitch": LaunchConfiguration("mount_pitch").perform(context),
        "so101_mount_yaw": LaunchConfiguration("mount_yaw").perform(context),
    }
    robot_description_xml = xacro.process_file(model_path, mappings=mappings).toxml()

    # joint_state_publisher in Humble expects a URDF file argument, while
    # robot_state_publisher accepts the XML as a parameter.
    with tempfile.NamedTemporaryFile(
        mode="w", prefix="amr_so101_", suffix=".urdf", delete=False
    ) as urdf_file:
        urdf_file.write(robot_description_xml)
        _TEMP_URDF_FILES.append(urdf_file.name)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_xml}],
    )

    if use_joint_state_gui.lower() in ("1", "true", "yes", "on"):
        joint_state_publisher = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
            arguments=[urdf_file.name],
        )
    else:
        joint_state_publisher = Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen",
            arguments=[urdf_file.name],
        )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_so101_amr",
        output="screen",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(use_rviz),
    )

    return [joint_state_publisher, robot_state_publisher, rviz]


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="AMR_DESCRIPTION_SHARE",
                value=FindPackageShare("amr_description"),
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz2 for visualizing the AMR with SO-101",
            ),
            DeclareLaunchArgument(
                "use_joint_state_gui",
                default_value="true",
                description="Launch joint_state_publisher_gui sliders",
            ),
            DeclareLaunchArgument(
                "mount_x",
                default_value="0.14",
                description="SO-101 mount x offset from base_link",
            ),
            DeclareLaunchArgument(
                "mount_y",
                default_value="0.00",
                description="SO-101 mount y offset from base_link",
            ),
            DeclareLaunchArgument(
                "mount_z",
                default_value="0.125",
                description="SO-101 mount z offset from base_link",
            ),
            DeclareLaunchArgument(
                "mount_roll",
                default_value="0.0",
                description="SO-101 mount roll",
            ),
            DeclareLaunchArgument(
                "mount_pitch",
                default_value="0.0",
                description="SO-101 mount pitch",
            ),
            DeclareLaunchArgument(
                "mount_yaw",
                default_value="0.0",
                description="SO-101 mount yaw",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
