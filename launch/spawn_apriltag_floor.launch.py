from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, RegisterEventHandler, EmitEvent, LogInfo, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_path('hippo_sim')
    default_urdf_path = package_path / 'models/apriltag/urdf/apriltag.xacro'

    launch_args = [
        DeclareLaunchArgument(name='urdf_path',
                              default_value=str(default_urdf_path),
                              description='Absolute path to apriltag .xacro'),
        DeclareLaunchArgument(name='tag_rows',
                              default_value='13',
                              description='Number of apriltag rows.'),
        DeclareLaunchArgument(name='tag_cols',
                              default_value='7',
                              description='Number of apriltag columns.'),
        DeclareLaunchArgument(name='tag_distance_x',
                              default_value='0.25',
                              description='Distance between tags in meters.'),
        DeclareLaunchArgument(name='tag_distance_y',
                              default_value='0.3',
                              description='Distance between tags in meters.'),
        DeclareLaunchArgument(
            name='tag_offset_x',
            default_value='0.2',
            description=('Offset of the first tag from the origin of the world '
                         'coordinate system.')),
        DeclareLaunchArgument(
            name='tag_offset_y',
            default_value='0.3',
            description=('Offset of the first tag from the origin of the world '
                         'coordinate system.')),
        DeclareLaunchArgument(
            name='tag_offset_z',
            default_value='-1.45',
            description=('Offset of the first tag from the origin of the world '
                         'coordinate system.')),
        DeclareLaunchArgument(
            name='tag_size',
            default_value='0.05',
            description='Tag size including the white border (i.e. 10x10 px)'),
        DeclareLaunchArgument(name='tag_thickness',
                              default_value='0.01',
                              description='Thickness of the tag model.'),
    ]

    tag_size = [
        ' --tag-size ',
        LaunchConfiguration('tag_size'),
    ]
    grid_size = [
        ' --grid-size ',
        LaunchConfiguration('tag_rows'),
        ' ',
        LaunchConfiguration('tag_cols'),
    ]
    offset = [
        ' --offset ',
        LaunchConfiguration('tag_offset_x'),
        ' ',
        LaunchConfiguration('tag_offset_y'),
        ' ',
        LaunchConfiguration('tag_offset_z'),
    ]
    distance = [
        ' --distance ',
        LaunchConfiguration('tag_distance_x'),
        ' ',
        LaunchConfiguration('tag_distance_y'),
    ]
    out_dir = [' --out-dir -']

    tag_poses = LaunchConfiguration(
        'tag_poses',
        default=Command([
            'ros2 run hippo_sim generate_tag_poses.py',
        ] + tag_size + grid_size + offset + distance + out_dir))

    tag_poses_arg = [' --tag-poses \'', tag_poses, '\'']
    sdf = LaunchConfiguration('sdf',
                              default=Command([
                                  'ros2 run hippo_sim generate_pool.py',
                              ] + tag_poses_arg))
    params = {'description': sdf}

    return LaunchDescription(launch_args + [
        Node(package='hippo_sim',
             executable='spawn',
             parameters=[params],
             arguments=['--param', 'description'],
             output='screen')
    ])
