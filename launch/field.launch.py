import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory("inrof2025_ros"), "launch")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    world = os.path.join(
        get_package_share_directory("inrof2025_ros"),
        "worlds",
        "field.world"
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    return ld

    # package_dir = get_package_share_directory("inrof2025_ros")
    # world_path = os.path.join(package_dir, "worlds", "field.world")

    # return LaunchDescription([
    #     launch_ros.actions.SetParameter(name="use_sim_time", value=True),

    #     launch.actions.ExecuteProcess(
    #         cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'
    #     ),
    # ])