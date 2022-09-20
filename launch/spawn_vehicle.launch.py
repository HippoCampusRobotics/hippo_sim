from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, RegisterEventHandler, EmitEvent, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_path('hippo_sim')
    default_model_path = package_path / 'models/hippo3/urdf/hippo3.xacro'
    default_vehicle_name = 'uuv00'

    model_launch_arg = DeclareLaunchArgument(
        name='model_path',
        default_value=str(default_model_path),
        description='Absolute model path')
    vehicle_name_launch_arg = DeclareLaunchArgument(
        name='vehicle_name',
        default_value=default_vehicle_name,
        description='Vehicle name used as namespace.')

    robot_description = LaunchConfiguration(
        'robot_description',
        default=Command([
            'ros2 run hippo_sim create_robot_description.py ', '--input ',
            LaunchConfiguration('model_path'), ' --mappings vehicle_name=',
            LaunchConfiguration('vehicle_name')
        ]))

    description = {'robot_description': robot_description}

    vehicle_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('vehicle_name')),
        Node(package='hippo_sim',
             executable='spawn',
             parameters=[description],
             arguments=[
                 '--param',
                 'robot_description',
                 '--x',
                 '1.0',
                 '--y',
                 '1.0',
                 '--z',
                 '-0.5',
                 '--Y',
                 '1.5708',
             ],
             output='screen'),
        Node(package='hippo_sim', executable='bridge', output='screen'),
    ])

    return LaunchDescription([
        model_launch_arg,
        vehicle_name_launch_arg,
        vehicle_group,
    ])
