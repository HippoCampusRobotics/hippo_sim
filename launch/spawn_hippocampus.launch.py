from ament_index_python.packages import get_package_share_path
from hippo_common import launch_helper
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def declare_args(launch_description: LaunchDescription) -> None:
    launch_helper.declare_vehicle_name_and_sim_time(
        launch_description=launch_description, use_sim_time_default='true'
    )
    action = DeclareLaunchArgument('vehicle_name')
    launch_description.add_action(action)
    action = DeclareLaunchArgument('use_sim_time')
    launch_description.add_action(action)
    action = DeclareLaunchArgument('use_vertical_camera', default_value='False')
    launch_description.add_action(action)
    action = DeclareLaunchArgument('use_acoustic_modem', default_value='False')
    launch_description.add_action(action)


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    package_path = get_package_share_path('hippo_sim')

    declare_args(launch_description=launch_description)

    ############################################################################
    # Camera Bridge
    ############################################################################
    action = launch_helper.create_camera_bridge(
        vehicle_name=LaunchConfiguration('vehicle_name'),
        camera_name='vertical_camera',
        use_camera=LaunchConfiguration('use_vertical_camera'),
        image_name='image_rect',
    )
    launch_description.add_action(action)

    ############################################################################
    # Spawn HippoCampus
    ############################################################################
    path = str(package_path / 'launch/spawn_vehicle.launch.py')
    source = PythonLaunchDescriptionSource(path)
    path = str(package_path / 'models/hippo3/urdf/hippo3.xacro')
    args = launch_helper.LaunchArgsDict()
    args.add(['use_acoustic_modem', 'use_vertical_camera'])
    args['model_path'] = path
    action = IncludeLaunchDescription(source, launch_arguments=args.items())
    launch_description.add_action(action)

    ############################################################################
    # HippoCampus Actuator Mixer
    ############################################################################

    args = launch_helper.LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    pkg = "hippo_control"
    config_file = launch_helper.config_file_path(
        pkg, "actuator_mixer/hippocampus_normalized_default.yaml"
    )
    
    action = DeclareLaunchArgument("mixer_path", default_value=config_file)
    launch_description.add_action(action)

    action = Node(
        package="hippo_control",
        executable="actuator_mixer_node",
        namespace=LaunchConfiguration("vehicle_name"),
        parameters=[
            args,
            LaunchConfiguration("mixer_path"),
        ],
        output="screen",
    )
    launch_description.add_action(action)

    return launch_description
