import os
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    desc_pkg = get_package_share_directory('robot_description')
    gazebo_pkg = get_package_share_directory('robot_gazebo')
    
    # Get ros_gz_sim package
    try:
        ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    except:
        ros_gz_sim_pkg = None

    xacro_file = os.path.join(desc_pkg, 'urdf', 'auto_vehicle.urdf.xacro')
    world_file = os.path.join(gazebo_pkg, 'world', 'world.world')
    
    robot_description = xacro.process_file(xacro_file).toxml()

    # Launch Gazebo using ros_gz_sim launch file if available
    if ros_gz_sim_pkg:
        gz_sim_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-r {world_file}'
            }.items()
        )
    else:
        # Fallback to direct gz sim command
        from launch.actions import ExecuteProcess
        gz_sim_launch = ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen',
        )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'my_robot',
            '-allow_renaming', 'true'
        ],
        output='screen',
    )
    
    # Spawn robot after robot_state_publisher starts
    spawn_robot_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[spawn_entity]
        )
    )

    return LaunchDescription([
        gz_sim_launch,
        robot_state_publisher,
        spawn_robot_handler,
    ])