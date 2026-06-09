import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('block_position_publisher')
    world_file = os.path.join(pkg_dir, 'worlds', 'block_world.sdf')
    
    # Start Gazebo Sim
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        name='gz_sim'
    )
    
    # Start the ros_gz_bridge for block pose (delayed to let Gazebo start)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pose_bridge',
        arguments=['/model/quadcopter/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose'],
        output='screen',
    )
   
    bridge_odom = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='quadcopter_odom_bridge',
    arguments=['/model/quadcopter/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
    output='screen',
    )
    
    # Start the force publisher node
    force_publisher = Node(
        package='block_position_publisher',
        executable='force_publisher',
        name='force_publisher',
        output='screen',
    )
    delayed_bridge = TimerAction(
        period=3.0,
        actions=[bridge,force_publisher,bridge_odom]
    )
    
    return LaunchDescription([
        gz_sim,
        delayed_bridge,
    ])
