# ROS2 python laucnh file for v-SLAM in Gazebo alignment
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Declare map input argument
    default_map_db_in_arg = './map/gazebo-480-20231012-38.msg'
    map_db_in_arg = DeclareLaunchArgument(
        'map_db_in',
        default_value=default_map_db_in_arg,
        description='Path to input map db file'
    )

    # Define Nodes
    stella_ros = Node(
        package='stella_vslam_ros',
        executable='run_slam',
        name='stella_ros',
        output='screen',
        arguments=[
            "--disable-mapping",
            "-v", "./orb_vocab.fbow",
            "-c", "./gazebo_480p.yaml",
            "--map-db-in", LaunchConfiguration('map_db_in')
        ]
    )

    ros_gz_camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='camera_bridge',
        arguments=["/camera/image_raw"]
    )

    # ExecuteProcess for launching MAVROS and align_vslam
    # mavros_launch = ExecuteProcess(
    #     cmd=['ros2', 'launch', 'mavros', 'px4.launch', 'fcu_url:=udp://:14540@127.0.0.1:14557'],
    #     output='screen'
    # )

    align_vslam = ExecuteProcess(
        cmd=['ros2', 'run', 'align_vslam', 'sim_align_vslam'],
        output='screen'
    )

    # Add actions
    ld.add_action(map_db_in_arg)
    #ld.add_action(mavros_launch)
    ld.add_action(stella_ros)
    ld.add_action(ros_gz_camera_bridge)
    ld.add_action(align_vslam)

    return ld
