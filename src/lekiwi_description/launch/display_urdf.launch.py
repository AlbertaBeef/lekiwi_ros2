import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_lekiwi_description = get_package_share_directory('lekiwi_description')

    # Path to URDF file
    urdf_file = os.path.join(pkg_lekiwi_description, 'urdf', 'lekiwi.urdf.xacro')

    # Launch argument for simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Process the xacro file and wrap in ParameterValue
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Robot State Publisher - publishes TF from URDF
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

    # Publish Zero Joints Node - publishes zero joint states for visualization
    publish_zero_joints_node = Node(
        package='lekiwi_description',
        executable='publish_zero_joints.py',
        name='publish_zero_joints',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        robot_state_publisher_node,
        publish_zero_joints_node,
    ])