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
    spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        parameters=[params],
        arguments=['-param', 'robot_description'],
        output='screen')

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v 3',
             str(world)],
        output='screen')

    return LaunchDescription([
        gazebo,
        spawn_entity
    ])
