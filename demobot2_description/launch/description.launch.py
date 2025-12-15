import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def _create_node(context, *args, **kwargs):
    # perform(context) 只能在有上下文的函数中使用（比如OpaqueFunction或Command来运行时处理）
    robot = LaunchConfiguration('robot_type').perform(context)
    pkg_path = FindPackageShare('demobot2_description').perform(context)
    urdf_path = os.path.join(pkg_path, 'urdf', robot, f'{robot}.urdf')

    if os.path.exists(urdf_path):
        with open(urdf_path, 'r') as f:
            robot_description = f.read()
    else:
        raise FileNotFoundError(f"No URDF/urdf found for robot '{robot}' in {os.path.join(pkg_path, 'urdf', robot)}")

    robot_state_publish_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description,
        }],
    )

    joint_state_publish_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    return [robot_state_publish_node, joint_state_publish_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
        name='robot_type',
        default_value='demobot',
        description='demobot, cubicbot, circularbot, etc.',
    ),
        DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    ),
        OpaqueFunction(function=_create_node),
    ])

