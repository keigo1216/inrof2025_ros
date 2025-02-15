from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro
import os

def generate_launch_description():
    x = 0.20
    y = 0.20
    z = 5
    theata = 1.57

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    package_dir = get_package_share_directory("inrof2025_ros")
    world = os.path.join(
        get_package_share_directory("inrof2025_ros"), "worlds", "field.world"
    )

    # load robot urdf file
    xacro_file = os.path.join(package_dir, "urdf", "diff.xacro.urdf")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            params,
            {"use_sim_time": use_sim_time}
        ]
    ) 

    # gazebo settings
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={"world": world}.items()
    )
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                    '-entity', 'trolley',
                    '-x', str(x),
                    '-y', str(y),
                    '-z', str(z),
                    '-Y', str(theata),
                ],
        output='screen'
    )
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_base_controller'],
        output='screen'
    )

    # rviz2 settings
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_diff_drive_base_controller],
            )
        ),
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        rviz_node
    ])


    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    # )
    
    # package_dir = get_package_share_directory("inrof2025_ros")

    # xacro_file = os.path.join(
    #     package_dir,
    #     'urdf',
    #     'diff.xacro.urdf'
    # )

    # doc = xacro.parse(open(xacro_file))
    # xacro.process_doc(doc)
    # params = {'robot_description': doc.toxml()}

    # node_robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[params]
    # )

    # spawn_entity = Node(
    #     package='gazebo_ros', executable='spawn_entity.py',
    #     arguments=[
    #         '-topic', 'robot_description',
    #         '-entity', 'diffbot'],
    #     output='screen'
    # )

    # load_joint_state_broadcaster = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'joint_state_broadcaster'],
    #     output='screen'
    # )

    # load_diff_drive_base_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'diff_drive_base_controller'],
    #     output='screen'
    # )

    # return LaunchDescription([
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=spawn_entity,
    #             on_exit=[load_joint_state_broadcaster],
    #         )
    #     ),
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=load_joint_state_broadcaster,
    #             on_exit=[load_diff_drive_base_controller],
    #         )
    #     ),
    #     gazebo,
    #     node_robot_state_publisher,
    #     spawn_entity,
    # ])