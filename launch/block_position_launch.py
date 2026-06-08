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
        name='block_pose_bridge',
        arguments=['/model/block/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose'],
        output='screen',
    )
   
    bridge_wrench = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='block_wrench_bridge',
        arguments=['/world/block_world/wrench@geometry_msgs/msg/Wrench[gz.msgs.EntityWrench'],
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
        actions=[bridge,bridge_wrench,force_publisher]
    )
    
    return LaunchDescription([
        gz_sim,
        delayed_bridge,
    ])
