from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.events import Shutdown
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare arguments
    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        description='Full path to the ROS2 bag file'
    )

    # Include mav_odom_launch.py
    mav_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(os.path.dirname(os.path.realpath(__file__)), 'online_02_execute_stella.launch.py')
        ),
    )

    # ros2 bag play command
    play_bag_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_file')],
        output='screen',
        name='play_bag_node'
    )

    # ros2 bag record command
    record_bag_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--all'],
        output='screen',
        name='record_bag_node'
    )

    # Define event handler to stop bag recording when bag playback is done
    stop_bag_recording = RegisterEventHandler(
        OnProcessExit(
            target_action=play_bag_node,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return LaunchDescription([
        bag_file_arg,
        mav_odom_launch,
        play_bag_node,
        record_bag_node,
        stop_bag_recording
    ])
