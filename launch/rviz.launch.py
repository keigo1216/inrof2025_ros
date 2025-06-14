from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, LogInfo, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
import os
import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = os.path.join(
        get_package_share_directory("inrof2025_ros"), "worlds", "field.world"
    )

    # robot parameters
    r = 0.03
    R = 0.15

    # robot initial position
    x = 0.20
    y = 0.20
    z = 5
    theata = 1.57

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={"world": world}.items()
    )
    package_dir = get_package_share_directory("inrof2025_ros")

    xacro_file = os.path.join(package_dir, "urdf", "robot_for_gazebo.xacro")
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

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    load_joint_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'velocity_controllers'],
        output='screen'
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    omuni_odometry_node = Node(
        package='inrof2025_ros',
        executable='omuni_odometry',
        parameters=[{
            "r": r,
            "R": R,
            "x": x,
            "y": y,
            "theata": theata,
            "use_sim_time": use_sim_time,
            "is_mapping": True,
        }],
        output="log"
    )

    omuni_drive_node = Node(
        package="inrof2025_ros",
        executable="omuni_drive_node",
        parameters=[{
            "r": r,
            "R": R,
            "use_sim_time": use_sim_time
        }],
        output="log"
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_velocity_controller]
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        rviz_node,
        omuni_odometry_node,
        omuni_drive_node,
    ])