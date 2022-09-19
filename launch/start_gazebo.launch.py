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
    world = package_path / 'world' / 'empty.sdf'
    pool_path = package_path / 'urdf/pool.xacro'

    pool_description = LaunchConfiguration(
        'pool_description',
        default=Command([
            'ros2 run hippo_sim create_robot_description.py ',
            '--input ',
            str(pool_path),
        ]))
    pool_params = {'pool_description': pool_description}

    gazebo = ExecuteProcess(cmd=['ign', 'gazebo', '-v 1',
                                 str(world)],
                            output='screen')

    return LaunchDescription([
        ExecuteProcess(cmd=[
            'ign',
            'gazebo',
            '-v 1',
            str(world),
        ],
                       output='screen'),
        Node(package='hippo_sim',
             executable='spawn',
             parameters=[pool_params],
             arguments=[
                 '--param',
                 'pool_description',
                 '--x',
                 '1.0',
                 '--y',
                 '2.0',
                 '--z',
                 '-1.5',
             ],
             output='screen'),
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=gazebo, on_exit=[EmitEvent(event=Shutdown())])),
    ])
