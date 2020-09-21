from launch import LaunchDescription
from launch_ros.actions import Node
import pathlib

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='segwayrmp2',
            namespace='segwayrmp1',
            executable='segway_node',
            name='segwayrmp_driver',
            parameters=[
                '/home/hax/rosws/src/segwayrmp2/config/params.yaml'
            ]
        )
    ])