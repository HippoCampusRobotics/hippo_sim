from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    package_path = get_package_share_path('hippo_sim')
    world = package_path / 'world' / 'empty.sdf'
    default_model_path = package_path / 'urdf/hippo2.xacro'
    model_arg = DeclareLaunchArgument(name='model',
                                      default_value=str(default_model_path),
                                      description='Absolute model path')
    robot_description = ParameterValue(Command(
        ['xacro', LaunchConfiguration('model')]),
                                       value_type=str)
    doc = xacro.parse(open(default_model_path))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'cartpole'],
        output='screen')

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo',
             str(world)],
        output='screen')

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])
