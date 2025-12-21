import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros. actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_lekiwi_description = get_package_share_directory('lekiwi_description')

    # Path to URDF file
    urdf_file = os.path.join(pkg_lekiwi_description, 'urdf', 'lekiwi.urdf.xacro')
    
    # Path to RViz config file (we'll create this)
    rviz_config_file = os.path. join(pkg_lekiwi_description, 'rviz2', 'display.rviz')

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
            'use_sim_time':  use_sim_time,
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

    # RViz2 Node - visualize the robot
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
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
        rviz_node,
    ])
