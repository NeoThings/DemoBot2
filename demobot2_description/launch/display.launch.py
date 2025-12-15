import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_type_arg = DeclareLaunchArgument(
        name='robot_type',
        default_value='demobot',
        description='Type of robot to launch (demobot, circularbot, cubicbot)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('demobot2_description'), 'launch', 'description.launch.py'])
        ),
        launch_arguments={
            'robot_type': LaunchConfiguration('robot_type'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    rviz_config_dir = os.path.join(
        get_package_share_directory('demobot2_description'),
        'rviz',
        'display.rviz')

    return LaunchDescription([
        robot_type_arg,
        use_sim_time_arg,
        robot_description_launch,
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir],
        )
    ])