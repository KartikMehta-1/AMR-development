from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    port = LaunchConfiguration("port")
    allowed_joints = LaunchConfiguration("allowed_joints")
    start_joint_state_merger = LaunchConfiguration("start_joint_state_merger")
    arm_joint_states_topic = LaunchConfiguration("arm_joint_states_topic")
    base_joint_states_topic = LaunchConfiguration("base_joint_states_topic")
    merged_joint_states_topic = LaunchConfiguration("merged_joint_states_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_fake_hardware", default_value="true"),
            DeclareLaunchArgument(
                "port",
                default_value="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A058493-if00",
            ),
            DeclareLaunchArgument("allowed_joints", default_value="so101_wrist_roll"),
            DeclareLaunchArgument("start_joint_state_merger", default_value="true"),
            DeclareLaunchArgument("arm_joint_states_topic", default_value="/so101/joint_states"),
            DeclareLaunchArgument("base_joint_states_topic", default_value="/amr/joint_states"),
            DeclareLaunchArgument("merged_joint_states_topic", default_value="/joint_states"),
            Node(
                package="amr_so101_driver",
                executable="so101_trajectory_bridge",
                name="so101_trajectory_bridge",
                output="screen",
                parameters=[
                    {
                        "use_fake_hardware": use_fake_hardware,
                        "port": port,
                        "allowed_joints_csv": allowed_joints,
                        "joint_states_topic": arm_joint_states_topic,
                    }
                ],
            ),
            Node(
                package="amr_so101_driver",
                executable="joint_state_merger",
                name="amr_joint_state_merger",
                output="screen",
                condition=IfCondition(start_joint_state_merger),
                parameters=[
                    {
                        "base_joint_states_topic": base_joint_states_topic,
                        "arm_joint_states_topic": arm_joint_states_topic,
                        "output_topic": merged_joint_states_topic,
                    }
                ],
            ),
        ]
    )
