from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='v4l2_camera',
                executable='v4l2_camera_node',
                parameters=[
                    {'video_device': '/dev/video4'},
                    {'pixel_format': 'YUYV'},
                    {'image_size': [400, 300]},
                    {'time_per_frame': [1, 15]},
                    {'output_encoding': 'rgb8'}
                ]
            ),
        ]
    )