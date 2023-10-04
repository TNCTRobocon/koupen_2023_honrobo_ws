from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_transport',
            executable='republish',
            arguments=["raw"],
            remappings=[
                ('in','/image_topic'),
                ('out','/image_topic_com')
            ]
        ),
        Node(
            package='image_transport',
            executable='republish',
            arguments=["raw"],
            remappings=[
                ('in','/depth_topic'),
                ('out','/depth_topic_com')
            ]
        ),

    ])