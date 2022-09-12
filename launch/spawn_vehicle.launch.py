from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import PushRosNamespace

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_path = get_package_share_path('hippo_sim')
    world = package_path / 'world' / 'empty.sdf'
    default_model_path = package_path / 'urdf/hippo2.xacro'
    default_vehicle_name = 'uuv00'

    model_launch_arg = DeclareLaunchArgument(
        name='model_path',
        default_value=str(default_model_path),
        description='Absolute model path')
    vehicle_name_launch_arg = DeclareLaunchArgument(
        name='vehicle_name',
        default_value=default_vehicle_name,
        description='Vehicle name used as namespace.')

    # PathJoinSubstitution(LaunchConfiguration('model_path'))
    robot_description = LaunchConfiguration(
        'robot_description',
        default=Command([
            'ros2 run hippo_sim create_robot_description.py ', '--input ',
            LaunchConfiguration('model_path'), ' --mappings vehicle_name=',
            LaunchConfiguration('vehicle_name')
        ]))
    params = {'robot_description': robot_description}

    vehicle_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('vehicle_name')),
        Node(package='hippo_sim',
             executable='spawn',
             parameters=[params],
             arguments=['--param', 'robot_description'],
             output='screen'),
        Node(package='hippo_sim', executable='bridge', output='screen'),
    ])

    gazebo = ExecuteProcess(cmd=['ign', 'gazebo', '-v 1',
                                 str(world)],
                            output='screen')

    return LaunchDescription(
        [model_launch_arg, vehicle_name_launch_arg, vehicle_group, gazebo])
