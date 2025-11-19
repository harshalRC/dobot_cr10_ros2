from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare argument
    declared_arguments = [
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Use fake hardware (true) or real robot (false)",
        )
    ]
    
    # Get argument value
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    
    # Build config
    moveit_config = (
        MoveItConfigsBuilder("dobot_cr10", package_name="dobot_cr10_moveit_config")
        .robot_description(mappings={"use_fake_hardware": use_fake_hardware})
        .to_moveit_configs()
    )
    
    return LaunchDescription(declared_arguments + [generate_demo_launch(moveit_config)])
