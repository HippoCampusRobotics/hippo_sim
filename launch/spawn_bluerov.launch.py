from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    package_path = get_package_share_path('hippo_sim')
    launch_path = str(package_path / 'launch/spawn_vehicle.launch.py')
    model_path = str(package_path / 'models/bluerov/urdf/bluerov.xacro')

    vehicle_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_path),
        launch_arguments={'model_path': model_path}.items())
    return LaunchDescription([vehicle_spawner])
