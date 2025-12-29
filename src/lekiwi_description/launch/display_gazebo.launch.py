import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_bme_gazebo_basics = get_package_share_directory('bme_gazebo_basics')
    gazebo_models_path, ignore_last_dir = os.path.split(pkg_bme_gazebo_basics)
    
    pkg_lekiwi_description = get_package_share_directory('lekiwi_description')
    lekiwi_model_path, ignore_last_dir = os.path.split(pkg_lekiwi_description)
    
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path + os.pathsep + lekiwi_model_path

    viewer1_name_arg = DeclareLaunchArgument(
        "viewer1_name",
        default_value="Hand Controller",
        description="Name of Image Viewer for hand_controller/annotations."
    )
    use_imshow_arg = DeclareLaunchArgument(
        "use_imshow",
        default_value="True",
        description="Use usbcam_subscriber to view annotated image."
    )
    
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='world.sdf',
        description='Name of the Gazebo world file to load'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='mogi_bot.urdf',
        description='Name of the URDF description to load'
    )

    # Path to URDF file
    urdf_file = os.path.join(pkg_lekiwi_description, 'urdf', 'lekiwi.urdf.xacro')
    #urdf_file = os.path.join(pkg_lekiwi_description, 'urdf', 'lekiwi.gazebo.xacro')
    
    # Path to RViz config file
    rviz_config_file = os.path. join(pkg_lekiwi_description, 'rviz2', 'display.rviz')

    # Launch argument for simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')


    # Process the xacro file and wrap in ParameterValue
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bme_gazebo_basics, 'launch', 'world.launch.py'),
        ),
        launch_arguments={
        'world': LaunchConfiguration('world'),
        }.items()
    )

    # Hand controller (most nodes are launch in *_part1.launch.py)
    hand_controller_viewer_node = Node(
        package='hand_controller',
        executable='usbcam_subscriber_node',
        name="hand_controller_annotations",
        parameters=[
            {"viewer_name":LaunchConfiguration("viewer1_name")}
        ],
        remappings=[
            ("image_raw", "hand_controller/image_annotated")
        ],
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('use_imshow'), '" == "True"']))
    )

    # Launch rviz
    #rviz_node = Node(
    #    package='rviz2',
    #    executable='rviz2',
    #    arguments=['-d', os.path.join(pkg_bme_gazebo_basics, 'rviz', 'rviz.rviz')],
    #    condition=IfCondition(LaunchConfiguration('rviz')),
    #    parameters=[
    #        {'use_sim_time': True},
    #    ]
    #)

    # Spawn the URDF model using the `/world/<world_name>/create` service
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "my_robot",
            "-topic", "robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.5", "-Y", "0.0"  # Initial spawn position
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ]
    )

    #robot_state_publisher_node = Node(
    #    package='robot_state_publisher',
    #    executable='robot_state_publisher',
    #    name='robot_state_publisher',
    #    output='screen',
    #    parameters=[
    #        {'robot_description': Command(['xacro', ' ', urdf_file_path]),
    #         'use_sim_time': True},
    #    ],
    #    remappings=[
    #        ('/tf', 'tf'),
    #        ('/tf_static', 'tf_static')
    #    ]
    #)

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

    # Node to bridge messages like /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V"
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ]
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(viewer1_name_arg)
    launchDescriptionObject.add_action(use_imshow_arg)    
    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(hand_controller_viewer_node)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(gz_bridge_node)
    
    return launchDescriptionObject    
