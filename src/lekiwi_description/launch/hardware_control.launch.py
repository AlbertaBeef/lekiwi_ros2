import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_lekiwi_description = get_package_share_directory('lekiwi_description')

    # Paths to files
    urdf_file = os.path.join(pkg_lekiwi_description, 'urdf', 'lekiwi.urdf.xacro')
    controller_config = os.path.join(pkg_lekiwi_description, 'config', 'omni_controllers.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Process the URDF
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            urdf_file
        ]),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }]
    )

    # Controller Manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            controller_config,
        ],
        output='screen',
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ]
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Omni Drive Controller Spawner
    # Wait for joint_state_broadcaster to start before spawning omni_drive_controller
    omni_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['omni_wheel_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Delay spawning omni_drive_controller until joint_state_broadcaster is loaded
    delay_omni_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[omni_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delay_omni_drive_controller_spawner,
    ])
