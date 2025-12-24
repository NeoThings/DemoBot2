import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    ld = LaunchDescription()
    
    rviz_config_dir = os.path.join(get_package_share_directory('demobot2_bringup'), 'rviz')
    worlds_file_dir = os.path.join(get_package_share_directory('demobot2_gazebo'), 'worlds')
    maps_file_dir = os.path.join(get_package_share_directory('demobot2_bringup'), 'maps')
    params_file_dir = os.path.join(get_package_share_directory('demobot2_bringup'), 'params')

    params = [
        ('robot_type', 'demobot', 'Type of robot to load in gazebo (demobot, circularbot, cubicbot)'),
        ('gui', 'true', 'Start Gazebo with GUI'),
        ('use_sim_time', 'true', 'Use Gazebo simulation time'),
        ('world', os.path.join(worlds_file_dir, 'square.world'), 'World file to load in Gazebo'),
        ('map', os.path.join(maps_file_dir, 'square.yaml'), 'map file to load in AMCL'),
        ('params_file', os.path.join(params_file_dir, 'nav2_params.yaml'), 'navigation parameters file'),
        ('slam', 'false', 'Run slam toolbox'),
        # ('estimation', 'true', 'Run robot_localization EKF'),
        ('localization', 'true', 'Run amcl and map_server'),
        ('namespace', '', 'Top-level namespace'),
        ('autostart', 'true', 'Automatically startup the nav2 stack'),
    ]
    
    for name, default, desc in params:
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=desc))

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('demobot2_gazebo'), '/launch', '/gazebo.launch.py']),
        launch_arguments={
            'world': LaunchConfiguration('world'),  # omitting to use default(empty world)
            'gui': LaunchConfiguration('gui'),
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('nav2_bringup'), '/launch', '/navigation_launch.py']),
        launch_arguments={
            # 由于navigation_launch中在给params_file默认值前,调用了RewrittenYaml,所以需要显示传入params_file,防止RewrittenYaml找不到文件报错
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': LaunchConfiguration('params_file'),
        }.items()
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('nav2_bringup'), '/launch', '/localization_launch.py']),
        condition=IfCondition(LaunchConfiguration('localization')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'map': LaunchConfiguration('map'),
        }.items()
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('slam_toolbox'), '/launch', '/online_async_launch.py']),
        condition=IfCondition(LaunchConfiguration('slam')),
        # launch_arguments={
        # }.items()
    )

    estimator_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(params_file_dir, 'ekf.yaml'), 
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(rviz_config_dir, 'bringup.rviz')],
    )

    ld.add_action(gazebo_launch)
    ld.add_action(navigation_launch)
    ld.add_action(localization_launch)
    ld.add_action(estimator_node)
    ld.add_action(slam_launch)
    ld.add_action(rviz_node)

    return ld 