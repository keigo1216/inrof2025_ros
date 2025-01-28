from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
import os
import xacro

def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    )
    package_dir = get_package_share_directory("inrof2025_ros")

    xacro_file = os.path.join(package_dir, "urdf", "trolley.xacro.urdf")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                    '-entity', 'trolley'],
        output='screen'
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    joint_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'velocity_controllers'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[joint_velocity_controller]
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])

    # package_dir = get_package_share_directory("inrof2025_ros")
    # urdf = os.path.join(package_dir, "urdf", "trolley.urdf")

    # with open(urdf, 'r') as file:
    #     robot_description_content = file.read()

    # robot_state_publisher_gui = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="both",
    #     parameters=[{'robot_description': robot_description_content}],
    # )

    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     name='joint_state_publisher_gui',
    #     output="screen"
    # )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    # )

    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    # )

    # spawn_entity = Node(
    #     package='gazebo_ros', executable='spawn_entity.py',
    #     arguments=[
    #         '-topic', 'robot_description',
    #         '-entity', 'diffbot'
    #     ],
    #     output='screen')

    # load_joint_state_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
    #     output='screen'
    # )
    # load_joint_group_velocity_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_group_velocity_controller'],
    #     output='screen'
    # )

    # nodes_to_run = [
    #     robot_state_publisher_gui,
    #     joint_state_publisher_node,
    #     rviz_node,
    #     # gazebo,
    #     load_joint_state_controller,
    #     # spawn_entity,
    #     # load_joint_group_velocity_controller
    # ]

    # return LaunchDescription(nodes_to_run)