import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch
import launch_ros.actions


def generate_launch_description():
    package_dir = get_package_share_directory("inrof2025_ros")
    world_path = os.path.join(package_dir, "worlds", "field.world")

    return LaunchDescription([
        launch_ros.actions.SetParameter(name="use_sim_time", value=True),

        launch.actions.ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'
        ),
    ])