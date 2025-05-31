from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
import xacro

def generate_launch_description():
    package_dir = get_package_share_directory("inrof2025_ros")

    # load robot urdf file
    xacro_file = os.path.join(package_dir, "urdf", "robot.xacro")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    map_server_config_path = os.path.join(
        package_dir,
        "map",
        "map.yaml"
    )
    lifecycle_nodes = ['map_server']

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[
            params,
        ]
    ) 

    map_server_cmd = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[{'yaml_filename': map_server_config_path}]
    )
    start_lifecycle_manager_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        emulate_tty=True,
        parameters=[
            {'autostart': True},
            {'node_names': lifecycle_nodes}
        ]
    )

    # tf transfromer
    static_from_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=['0', '0', '-0.255', '0', '0', '0', 'map', 'odom']
    )

    mcl_node = Node(
        package="inrof2025_ros",
        executable="mcl_node",
        output="screen"
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    joy2Vel_node = Node(
        package="inrof2025_ros",
        executable="joy2vel",
        name="joy2vel",
        output="screen"
    )

    # cmd_velをキャッチして、uartに流すプログラムが必要
    return LaunchDescription([
        node_robot_state_publisher,
        map_server_cmd,
        start_lifecycle_manager_cmd,
        static_from_map_to_odom,
        mcl_node,
        joy_node,
        joy2Vel_node
    ])