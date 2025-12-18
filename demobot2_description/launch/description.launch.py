import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def _create_node(context, *args, **kwargs):
    # perform(context) 只能在有上下文的函数中使用（比如OpaqueFunction或Command来运行时处理）
    pkg_path = FindPackageShare('demobot2_description').perform(context)
    run_sim = LaunchConfiguration('use_sim_time').perform(context)
    robot = LaunchConfiguration('robot_type').perform(context)
    print(f"Generate URDF description with run simulation: {run_sim}, robot type: {robot}")

    if run_sim.lower() in ['true', '1', 't', 'yes', 'y']:    
        urdf_path = os.path.join(pkg_path, 'urdf', robot, f'{robot}_gazebo.urdf')
    else:
        urdf_path = os.path.join(pkg_path, 'urdf', robot, f'{robot}_urdf_fixed.urdf')

    if os.path.exists(urdf_path):
        with open(urdf_path, 'r') as f:
            robot_description = f.read()
    else:
        raise FileNotFoundError(f"No URDF/urdf found for robot '{robot}' in {urdf_path}")

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

    # joint_state_publish_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    # )
    # return [robot_state_publish_node, joint_state_publish_node]

    # OpaqueFunction must return a list (or iterable) of entities/actions.
    return [robot_state_publish_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
        name='robot_type',
        default_value='demobot',
        description='Type of robot to launch (demobot, cubicbot, circularbot, etc.)',
    ),
        DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    ),
        OpaqueFunction(function=_create_node),
    ])

