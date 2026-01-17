import os
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    desc_pkg = get_package_share_directory('robot_description')
    gazebo_pkg = get_package_share_directory('robot_gazebo')

    xacro_file = os.path.join(desc_pkg, 'urdf', 'auto_vehicle.urdf.xacro')
    world_file = os.path.join(gazebo_pkg, 'world', 'world.world')
    
    robot_description = xacro.process_file(xacro_file).toxml()

    # Launch Gazebo Harmonic in server mode (no GUI)
    gz_sim_launch = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        shell=False
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # Spawn robot with delay to allow Gazebo to fully start
    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', '/robot_description',
                    '-name', 'my_robot',
                    '-allow_renaming', 'true'
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        gz_sim_launch,
        robot_state_publisher,
        spawn_entity,
    ])