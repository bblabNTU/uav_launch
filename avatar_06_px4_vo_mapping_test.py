from launch import LaunchDescription
from launch_ros.actions import Node
# For environment variables
from launch.actions import SetEnvironmentVariable

# Import other 'launch files'
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory
from datetime import datetime

def generate_launch_description():
    ld = LaunchDescription()
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
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id','odom','--child-frame-id','base_link'],
        name='odom_to_base_link'
    )
    base_link_to_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '-1.5708', '--pitch', '0', '--roll', '-1.5708',
                     '--frame-id','base_link','--child-frame-id','camera_frame'],
        name='base_link_to_camera_link'
    )
    
    # ros2 run stella_vslam_ros run_slam -v ./orb_vocab.fbow -c ./runcam_720p.yaml --map-db-out ./runcam_720_0420.msg
    # stella_ros = Node(
    #     package='stella_vslam_ros',
    #     executable='run_slam',
    #     name='stella_ros',
    #     output="screen",
    #     remappings=[("/stella_ros/camera_pose","/mavros/odometry/out")],
    #                #("/camera/image_raw", "/cam0/image_raw")]
        # arguments=["-v","./orb_vocab.fbow","-c","./runcam_720p.yaml",
        #            "--map-db-out","./runcam_720_0420.msg"
    #                #"--ros-args","-p","publish_tf:=false"
    #                ]
    #)

    # Generate the timestamp string
    timestamp = datetime.now().strftime("%Y%m%d-%H%M")

    # Create the modified argument
    map_argument = "../data/map/avatar720-" + timestamp + ".msg"

    # Update the arguments list for the stella_ros node
    stella_ros_arguments = [
        "-v", "./orb_vocab.fbow",
        "-c", "./avatar_720p.yaml",
        "--map-db-out", map_argument
    ]

    stella_ros = Node(
        package='stella_vslam_ros',
        executable='run_slam',
        name='stella_ros',
        output="screen",
        # remappings=[("/stella_ros/camera_pose","/mavros/vision_pose/pose")],
        # remappings=[("/stella_ros/camera_pose","/mavros/odometry/out")],
        #arguments=["-v","./orb_vocab.fbow","-c","./runcam_720p.yaml","--map-db-out","./runcam_720_0420.msg"],
        arguments=stella_ros_arguments,
        parameters=[
            {"odom_frame": "odom"},
            {"map_frame": "map"},
            {"base_link": "base_link"},
            {"camera_frame": "camera_frame"},
            {"publish_tf": True},
            {"publish_keyframes": True},
            {"transform_tolerance": 0.5}
        ]
        
    )
    
    odometry_processor = Node(
        package='offb_control_mavros',
        executable='odometry_processor',
        name='odometry_processor',
        output="screen",
        parameters=[
            {"source_mode": "vision"},
            {"max_allowed_jump": 1.0}
        ]
    )
    
    #ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 ROS_DOMAIN_ID=0
    # micro_ros = Node(
    #     package='micro_ros_agent',
    #     executable='micro_ros_agent',
    #     name='micro_ros',
    #     arguments=["udp4","--port","8888","ROS_DOMAIN_ID=0"]
    # )

    # #ros2 run ros_gz_image image_bridge /camera
    # ros_gz_camera_bridge = Node(
    #     package='ros_gz_image',
    #     executable='image_bridge',
    #     name='camera_bridge',
    #     arguments=["/camera"]
    # )

    # #ros2 run px4_ros_com offboard_control
    # px4_offboard = Node(
    #     package='px4_ros_com',
    #     executable='offboard_control',
    #     name='offboard_takeoff'
    # )

    ld.add_action(stella_ros)
    #ld.add_action(tf2_ros)
    ld.add_action(odom_to_base_link)
    ld.add_action(base_link_to_camera_link)
    ld.add_action(odometry_processor)
    #ld.add_action(ros_gz_camera_bridge)
    # ld.add_action(px4_offboard)
    return ld
