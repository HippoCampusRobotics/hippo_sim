from email.policy import default
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, RegisterEventHandler, EmitEvent, LogInfo
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_path('hippo_sim')
    default_urdf_path = package_path / 'urdf/apriltag/apriltag.xacro'

    urdf_path_launch_arg = DeclareLaunchArgument(
        name='urdf_path',
        default_value=str(default_urdf_path),
        description='Absolute path to apriltag .xacro')

    SIZE_X = 1
    SIZE_Y = 1
    SIZE_Z = 0.1
    tag_id = 103
    mappings = f' --mappings size_x={SIZE_X} size_y={SIZE_Y} size_z={SIZE_Z} tag_id={tag_id}'
    description = LaunchConfiguration(
        'description',
        default=Command([
            'ros2 run hippo_sim create_robot_description.py ',
            '--input ',
            LaunchConfiguration('urdf_path'),
            mappings,
        ]))
    params = {'description': description}

    return LaunchDescription([
        urdf_path_launch_arg,
        Node(package='hippo_sim',
             executable='spawn',
             parameters=[params],
             arguments=['--param', 'description'],
             output='screen'),
        LogInfo(msg=description)
    ])
