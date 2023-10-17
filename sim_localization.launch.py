from launch import LaunchDescription
from launch_ros.actions import Node
# For environment variables
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import time

# Import other 'launch files'
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # Map input
    # default_map_db_in_arg = './map/gazebo-480-' + time.strftime("%Y%m%d-%M") + '.msg'
    default_map_db_in_arg = '../dat/map/gazebo-480-20231012-38.msg'
    map_db_in_arg = DeclareLaunchArgument(
        'map_db_in',
        default_value=default_map_db_in_arg,
        description='Path to input map db file')
    
    # Set ROS Domain ID
    # ld.add_action(SetEnvironmentVariable('ROS_DOMAIN_ID', '101'))
    
    # TF                                  
    tf2_ros = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        name='odom_to_odom'
    )

    odom_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        name='odom_to_base_link'
    )
    base_link_to_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '-1.57', '--pitch', '0', '--roll', '-1.57',
                '--frame-id','base_link','--child-frame-id','x500_vision_0/camera_link/camera'],
        name='base_link_to_camera_link'
    )
    x500_to_camera_frame= Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'x500_vision_0/camera_link/camera', 'camera_frame'],
        name='base_link_to_camera_link'
    )

    
    
    # ros2 run stella_vslam_ros run_slam -v ./orb_vocab.fbow -c ./runcam_720p.yaml --map-db-out ./runcam_720_0420.msg
    stella_ros = Node(
        package='stella_vslam_ros',
        executable='run_slam',
        name='stella_ros',
        output="screen",
        #remappings=[("/stella_ros/camera_pose","/mavros/odometry/out")],
        arguments=[ "--disable-mapping",
                    "-v","./orb_vocab.fbow",
                    "-c","./gazebo_480p.yaml",
                    "--map-db-in",LaunchConfiguration('map_db_in')
                    #"--ros-args","-r", "/camera/image_raw:=/cam0/image_raw"
                    #"--ros-args","-p","publish_tf:=false"
                    ]
    )
    
    #ros2 run ros_gz_image image_bridge /camera
    ros_gz_camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='camera_bridge',
        arguments=["/camera/image_raw"]
    )

    ld = LaunchDescription()
    ld.add_action(map_db_in_arg)
    ld.add_action(stella_ros)
    # ld.add_action(tf2_ros)
    ld.add_action(odom_to_base_link)
    ld.add_action(base_link_to_camera_link)
    ld.add_action(x500_to_camera_frame) 
    ld.add_action(ros_gz_camera_bridge)

    # ld.add_action(px4_offboard)
    return ld
