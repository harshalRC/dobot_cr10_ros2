from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Use fake hardware for testing'
    )
    
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.5.1',
        description='IP address of the Dobot CR10'
    )
    
    robot_port_arg = DeclareLaunchArgument(
        'robot_port',
        default_value='29999',
        description='Port of the Dobot CR10'
    )

    # Get launch configurations
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    robot_ip = LaunchConfiguration('robot_ip')
    robot_port = LaunchConfiguration('robot_port')

    # Robot description with parameters
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('dobot_cr10_description'), 'urdf', 'dobot_cr10.urdf.xacro']
            ),
            ' use_fake_hardware:=', use_fake_hardware,
            ' robot_ip:=', robot_ip,
            ' robot_port:=', robot_port,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # Controller manager config
    controller_config = PathJoinSubstitution(
        [FindPackageShare('dobot_cr10_description'), 'config', 'dobot_controllers.yaml']
    )

    # Controller Manager Node
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Joint State Broadcaster (spawner)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Arm Controller (spawner)
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Use TimerAction to delay spawner execution
    delay_joint_state_broadcaster = TimerAction(
        period=2.0,
        actions=[joint_state_broadcaster_spawner]
    )

    delay_arm_controller = TimerAction(
        period=3.0,
        actions=[arm_controller_spawner]
    )

    # RViz
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('dobot_cr10_description'), 'rviz', 'dobot_cr10.rviz']
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # # Joint State Publisher GUI (only when using fake hardware)
    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     output='screen',
    #     condition=IfCondition(use_fake_hardware)
    # )

    # Build launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(use_fake_hardware_arg)
    ld.add_action(robot_ip_arg)
    ld.add_action(robot_port_arg)
    
    # Add nodes
    ld.add_action(controller_manager_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(delay_joint_state_broadcaster)
    ld.add_action(delay_arm_controller)
    ld.add_action(rviz_node)
    #ld.add_action(joint_state_publisher_gui_node)
    
    return ld
