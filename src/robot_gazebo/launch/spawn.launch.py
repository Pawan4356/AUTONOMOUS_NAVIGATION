import os
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    main_pkg = get_package_share_directory('robot_gazebo')

    xacro_file = os.path.join(main_pkg, 'models', 'auto_vehicle', 'auto_vehicle.urdf.xacro')
    world_file = os.path.join(main_pkg, 'models', 'world', 'world.sdf')
    # world_file = os.path.join(main_pkg, 'models', 'world', 'industrial-warehouse.sdf')
    # world_file = os.path.join(main_pkg, 'models', 'world', 'tugbot_depot.sdf')
    # world_file = os.path.join(main_pkg, 'models', 'world', 'city', 'model.sdf')
    
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
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'my_robot',
            '-x', '0',
            '-y', '-8.75',
            '-z', '0',
        ],
        output='screen',
    )

    # ROS-Gazebo Bridge
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ]
    )

    return LaunchDescription([
        gz_sim_launch,
        robot_state_publisher,
        spawn_entity,
        gz_bridge_node,
    ])