from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # Map input
    default_mapdb_in_arg = '../data/map/runcam720-20210630-1720.msg'
    map_db_in_arg = DeclareLaunchArgument(
        'map_db_in',
        default_value= default_mapdb_in_arg,
        description='Path to map, map_db_in:=../data/map/...'
    )
     # cam2image command
    cam2image_node = Node(
        package='image_tools',
        executable='cam2image',
        name='cam2image_node',
        parameters=[{
            'device_id': 2,
            'width': 1280,
            'height': 720,
            'frequency': 30.0
        }],
        output='screen'
    )

    # republish command
    republish_node = Node(
        package='image_transport',
        executable='republish',
        name='republish_node',
        arguments=['raw', 'in:=image', 'raw', 'out:=/camera/image_raw'],
        output='screen'
    )

    # TF                                                                  
    tf2_ros = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom_ned'],
        name='odom_to_odom'
    )

    odom_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '1', '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id','odom','--child-frame-id','base_link'],
        name='odom_to_base_link'
    )
    # --yaw -1.5708 --roll -1.5708 --pitch 0 (align with map)
    base_link_to_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '-1.5708', 
                     '--frame-id','base_link','--child-frame-id','camera_frame'],
        name='base_link_to_camera_link'
    )

    # Node for Stella SLAM ROS
    stella_ros = Node(
        package='stella_vslam_ros',
        executable='run_slam',
        name='stella_ros',
        output="screen",
        #remappings=[("/stella_ros/camera_pose","/mavros/odometry/out")],
        arguments=["--disable-mapping",
                   "-v","./orb_vocab.fbow",
                   "-c","./runcam_720p.yaml",
                   "--map-db-in",LaunchConfiguration('db_in')],
        parameters=[
            {"odom_frame": "odom"},
            {"map_frame": "map"},
            {"base_link": "base_footprint"},
            {"camera_frame": "camera_frame"},
            #{"publish_tf": True},
            {"publish_keyframes": True},
            {"transform_tolerance": 0.5}
        ]
    )
    # Node for Calibrated pose
    calibrated_pose = Node(
        package='align_vslam',
        executable='calibrated_pose',
        name='calibrated_pose',
        parameters=[
            {"yaml_path": "../data/align_param.yaml"}
        ]
    )

    return LaunchDescription([
        cam2image_node,
        republish_node,
        map_db_in_arg,
        odom_to_base_link,
        base_link_to_camera_link,
        stella_ros,
        calibrated_pose
    ])
