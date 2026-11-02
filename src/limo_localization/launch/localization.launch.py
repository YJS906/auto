from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = Path(get_package_share_directory('limo_localization'))
    sim = Path(get_package_share_directory('limo_autorace_sim'))
    defaults = {
        'config': str(package / 'config/localization.yaml'),
        'map_image': str(sim / 'models/autorace_track/materials/textures/track.png'),
        'projection': 'ipm', 'matcher': 'distance', 'use_sim_time': 'true',
        'namespace': '', 'publish_tf': 'true',
        'global_initialization': 'true',
        'image': '/camera/color/image_raw', 'camera_info': '/camera/color/camera_info',
        'depth': '/camera/depth/image_raw', 'depth_info': '/camera/depth/camera_info',
        'odom': '/odom', 'imu': '/imu',
    }
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] + [
            Node(package='limo_localization', executable='road_marking_localizer',
                 namespace=LaunchConfiguration('namespace'),
                 output='screen', parameters=[LaunchConfiguration('config'), {
                     'map_image': LaunchConfiguration('map_image'),
                     'projection': LaunchConfiguration('projection'),
                     'matcher': LaunchConfiguration('matcher'),
                     'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                     'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
                     'global_initialization': ParameterValue(LaunchConfiguration('global_initialization'), value_type=bool),
                 }], remappings=[(k, LaunchConfiguration(k)) for k in
                                 ('image', 'camera_info', 'depth', 'depth_info', 'odom', 'imu')])])
