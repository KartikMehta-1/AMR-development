import os
import tempfile

import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_TEMP_URDF_FILES = []


def load_yaml(package_name, relative_path):
    package_share = get_package_share_directory(package_name)
    path = os.path.join(package_share, relative_path)
    with open(path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def load_text(package_name, relative_path):
    package_share = get_package_share_directory(package_name)
    path = os.path.join(package_share, relative_path)
    with open(path, "r", encoding="utf-8") as file:
        return file.read()


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration("use_rviz")
    use_joint_state_gui = LaunchConfiguration("use_joint_state_gui")
    use_so101_driver = LaunchConfiguration("use_so101_driver")

    description_share = get_package_share_directory("amr_description")
    config_share = get_package_share_directory("amr_so101_moveit_config")
    driver_share = get_package_share_directory("amr_so101_driver")
    model_path = os.path.join(description_share, "urdf", "amr.urdf.xacro")
    rviz_config_path = os.path.join(config_share, "rviz", "moveit.rviz")
    driver_launch_path = os.path.join(driver_share, "launch", "so101_driver.launch.py")

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
    robot_description = {"robot_description": robot_description_xml}
    robot_description_semantic = {
        "robot_description_semantic": load_text(
            "amr_so101_moveit_config", "config/amr_so101.srdf"
        )
    }
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "amr_so101_moveit_config", "config/kinematics.yaml"
        )
    }
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            "amr_so101_moveit_config", "config/joint_limits.yaml"
        )
    }
    ompl_planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": load_yaml("amr_so101_moveit_config", "config/ompl_planning.yaml"),
    }
    moveit_controllers = load_yaml(
        "amr_so101_moveit_config", "config/moveit_controllers.yaml"
    )
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    moveit_parameters = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        ompl_planning_pipeline,
        moveit_controllers,
        planning_scene_monitor_parameters,
    ]

    with tempfile.NamedTemporaryFile(
        mode="w", prefix="amr_so101_moveit_", suffix=".urdf", delete=False
    ) as urdf_file:
        urdf_file.write(robot_description_xml)
        _TEMP_URDF_FILES.append(urdf_file.name)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        arguments=[urdf_file.name],
        condition=IfCondition(use_joint_state_gui),
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=moveit_parameters,
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_amr_so101_moveit",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=moveit_parameters,
        condition=IfCondition(use_rviz),
    )

    so101_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(driver_launch_path),
        launch_arguments={
            "use_fake_hardware": LaunchConfiguration("driver_use_fake_hardware"),
            "port": LaunchConfiguration("so101_port"),
            "allowed_joints": LaunchConfiguration("driver_allowed_joints"),
            "start_joint_state_merger": LaunchConfiguration("start_joint_state_merger"),
            "base_joint_states_topic": LaunchConfiguration("base_joint_states_topic"),
            "arm_joint_states_topic": LaunchConfiguration("arm_joint_states_topic"),
            "merged_joint_states_topic": LaunchConfiguration("merged_joint_states_topic"),
        }.items(),
        condition=IfCondition(use_so101_driver),
    )

    return [robot_state_publisher, joint_state_publisher_gui, so101_driver, move_group, rviz]


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="0"),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz with the MoveIt MotionPlanning display",
            ),
            DeclareLaunchArgument(
                "use_joint_state_gui",
                default_value="true",
                description="Launch joint_state_publisher_gui sliders",
            ),
            DeclareLaunchArgument(
                "use_so101_driver",
                default_value="false",
                description="Launch the SO-101 FollowJointTrajectory bridge",
            ),
            DeclareLaunchArgument(
                "driver_use_fake_hardware",
                default_value="true",
                description="Run the SO-101 bridge without touching physical motors",
            ),
            DeclareLaunchArgument(
                "so101_port",
                default_value="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A058493-if00",
            ),
            DeclareLaunchArgument(
                "driver_allowed_joints",
                default_value="so101_wrist_roll",
                description="Comma-separated SO-101 joints allowed to execute",
            ),
            DeclareLaunchArgument(
                "start_joint_state_merger",
                default_value="true",
                description="Merge AMR/base and SO-101 joint states into /joint_states",
            ),
            DeclareLaunchArgument("base_joint_states_topic", default_value="/amr/joint_states"),
            DeclareLaunchArgument("arm_joint_states_topic", default_value="/so101/joint_states"),
            DeclareLaunchArgument("merged_joint_states_topic", default_value="/joint_states"),
            DeclareLaunchArgument("mount_x", default_value="0.14"),
            DeclareLaunchArgument("mount_y", default_value="0.00"),
            DeclareLaunchArgument("mount_z", default_value="0.125"),
            DeclareLaunchArgument("mount_roll", default_value="0.0"),
            DeclareLaunchArgument("mount_pitch", default_value="0.0"),
            DeclareLaunchArgument("mount_yaw", default_value="0.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
