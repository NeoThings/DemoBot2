import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    ld = LaunchDescription()
    
    rviz_config_dir = os.path.join(get_package_share_directory('demobot2_bringup'), 'rviz')

    params = [
        ('robot_type', 'circularbot', 'Type of robot to load in gazebo (demobot, circularbot, cubicbot)'),
        ('world_name', 'square', 'world to load'),
        ('map_name', 'square', 'map to load'),
        ('gui', 'true', 'Start Gazebo with GUI'),
        ('use_sim_time', 'true', 'Use Gazebo simulation time'),
        ('slam', 'false', 'Run slam toolbox'),
        ('estimation', 'true', 'Run robot_localization EKF'),
        ('namespace', '', 'Top-level namespace'),
        ('autostart', 'true', 'Automatically startup the nav2 stack'),
    ]
    
    for name, default, desc in params:
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=desc))
    
    nav_params_file = PathJoinSubstitution(
        [FindPackageShare('demobot2_bringup'), 'params', 
         LaunchConfiguration('robot_type'), 'nav2_params.yaml'])
    ekf_params_file = PathJoinSubstitution(
        [FindPackageShare('demobot2_bringup'), 'params', 
         LaunchConfiguration('robot_type'), 'ekf.yaml'])
    
    world_file_name = (LaunchConfiguration('world_name'), '.world')
    world_file = PathJoinSubstitution(
        [FindPackageShare('demobot2_gazebo'), 'worlds', world_file_name])
    
    map_file_name = (LaunchConfiguration('map_name'), '.yaml')
    map_file = PathJoinSubstitution(
        [FindPackageShare('demobot2_bringup'), 'maps', 
         LaunchConfiguration('map_name'), map_file_name])
    
    keepout_mask_name = (LaunchConfiguration('map_name'), '_keepout.yaml')
    keepout_mask_file = PathJoinSubstitution(
        [FindPackageShare('demobot2_bringup'), 'maps', 
         LaunchConfiguration('map_name'), keepout_mask_name])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('demobot2_gazebo'), '/launch', '/gazebo.launch.py']),
        launch_arguments={
            'world': world_file,  # omitting to use default(empty world)
            'gui': LaunchConfiguration('gui'),
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('demobot2_bringup'), '/launch', '/navigation_launch.py']),
        launch_arguments={
            # 由于navigation_launch中在给params_file默认值前,调用了RewrittenYaml,所以需要显示传入params_file,防止RewrittenYaml找不到文件报错
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': nav_params_file,
        }.items()
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('demobot2_bringup'), '/launch', '/localization_launch.py']),
        condition=UnlessCondition(LaunchConfiguration('slam')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav_params_file,
            'autostart': LaunchConfiguration('autostart'),
            'map': map_file,
        }.items()
    )

    costmap_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('demobot2_bringup'), '/launch', '/costmap_filter_launch.py']),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': nav_params_file,
            'keepout_mask': keepout_mask_file,
        }.items()
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('slam_toolbox'), '/launch', '/online_async_launch.py']),
        condition=IfCondition(LaunchConfiguration('slam')),
        # launch_arguments={
        # }.items()
    )

    estimator_node = Node(
        condition=IfCondition(LaunchConfiguration('estimation')),
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, 
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(rviz_config_dir, 'bringup.rviz')],
    )

    ld.add_action(rviz_node)
    ld.add_action(gazebo_launch)
    ld.add_action(navigation_launch)
    ld.add_action(costmap_filter_launch)
    ld.add_action(estimator_node)
    ld.add_action(slam_launch)
    ld.add_action(TimerAction(
        period=5.0,  # Delay to let sim time flow first
        actions=[localization_launch],
    ))
    
    return ld 