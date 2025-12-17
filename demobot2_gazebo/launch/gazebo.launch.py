import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # description_launch_dir = PathJoinSubstitution([FindPackageShare('demobot2_description'), 'launch'])
    urdf_path = os.path.join(get_package_share_directory('demobot2_description'), 'urdf', 'demobot', 'demobot_urdf.urdf')

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Start Gazebo with GUI'
    )

    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=PathJoinSubstitution([FindPackageShare('demobot2_gazebo'), 'worlds', 'square.world']),
        description='World file to load in Gazebo'
    )

    run_gzserver_arg = DeclareLaunchArgument(
        name='server', 
        default_value='true',
        description='Set to "false" not to run gzserver.'
    )

    robot_type_arg = DeclareLaunchArgument(
        name='robot_type',
        default_value='cubicbot',
        description='Type of robot to load in gazebo (demobot, circularbot, cubicbot)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use Gazebo simulation time'
    )

    # gazebo.launch.py中ThisLaunchFileDir会导致问题，直接启用gzserver和gzclient
    # empty_world_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([gazebo_launch_dir, 'gazebo.launch.py'])
    #     ),
    #     launch_arguments={
    #         'gui': LaunchConfiguration('gui'),
    #         'pause': 'true',
    #     }.items()
    # )

    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('gazebo_ros'), '/launch', '/gzserver.launch.py']),
        condition=IfCondition(LaunchConfiguration('server')),
        launch_arguments={
            'world': LaunchConfiguration('world'),  # omitting to use default(empty world)
            'pause': 'true',
        }.items()
    )

    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('gazebo_ros'), '/launch', '/gzclient.launch.py']),
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('demobot2_description'), '/launch', '/description.launch.py']),
        launch_arguments={
            'robot_type': LaunchConfiguration('robot_type'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description', 
            # '-file', urdf_path, 
            '-entity', LaunchConfiguration('robot_type'), 
            '-x', '0.0', 
            '-y', '0.0', 
            '-z', '0.5', 
            '-unpause',
        ],
        output='screen',
    )

    rviz_config_dir = os.path.join(
        get_package_share_directory('demobot2_gazebo'),
        'rviz',
        'gazebo.rviz')

    return LaunchDescription([
        gui_arg,
        robot_type_arg,
        world_arg,
        use_sim_time_arg,
        run_gzserver_arg,
        gzserver_launch,
        gzclient_launch,
        robot_description_launch,
        spawn_node,
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir],
        )
    ])