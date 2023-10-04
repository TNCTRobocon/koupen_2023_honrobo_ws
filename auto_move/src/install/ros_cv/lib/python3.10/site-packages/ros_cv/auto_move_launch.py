from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy_linux',
            executable='joy_linux_node',
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
        ),
        Node(
            package='ros_joy',
            executable='ros_joy',
            prefix="xterm -e",
            output="screen",
        ),
        Node(
            package='ros_cv',
            executable='ros_cv',
            prefix="xterm -e",
            output="screen",
        ),
        Node(
            package='image_transport',
            executable='republish',
            arguments=["raw"],
            remappings=[
                ('in','/converted_img')
            ]
        ),

    ])