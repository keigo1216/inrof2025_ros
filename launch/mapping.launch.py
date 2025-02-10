import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    package_dir = get_package_share_directory("inrof2025_ros")
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_params = os.path.join(package_dir, "config", 'slam_settings.yaml')

    return LaunchDescription([
        Node(
            parameters=[
            {"slam_params_file": slam_params},
            {'use_sim_time': use_sim_time}
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'),
    ])