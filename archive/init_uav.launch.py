'''
	# This is the lauch file to run micro_ros & gazebo image bridge
'''

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    #ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 ROS_DOMAIN_ID=0
    micro_ros = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros',
        arguments=["udp4","--port","8888","ROS_DOMAIN_ID=0"]
    )

    #ros2 run ros_gz_image image_bridge /camera
    ros_gz_camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='camera_bridge',
        arguments=["/camera/image_raw"]
    )

    ld.add_action(micro_ros)
    ld.add_action(ros_gz_camera_bridge)
    return ld
