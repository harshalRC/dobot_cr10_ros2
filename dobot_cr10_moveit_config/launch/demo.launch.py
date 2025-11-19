from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.5.1",
            description="IP address of the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_port",
            default_value="29999",
            description="Port number of the robot",
        )
    )

    # Get launch arguments
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    robot_port = LaunchConfiguration("robot_port")

    # Build MoveIt config with mappings
    moveit_config = (
        MoveItConfigsBuilder("dobot_cr10", package_name="dobot_cr10_moveit_config")
        .robot_description(
            mappings={
                "use_fake_hardware": use_fake_hardware,
                "robot_ip": robot_ip,
                "robot_port": robot_port,
            }
        )
        .to_moveit_configs()
    )

    return LaunchDescription(declared_arguments + [generate_demo_launch(moveit_config)])

