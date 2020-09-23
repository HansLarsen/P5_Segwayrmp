import pathlib
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

	ld = LaunchDescription()

	config = os.path.join(
		get_package_share_directory('segwayrmp2'),
		'config',
		'params.yaml'
		)


	node=Node(
		package='segwayrmp2',
		namespace='segwayrmp1',
		executable='segway_node',
		name='segwayrmp_driver',
		parameters=[config]
	)

	ld.add_action(node)
	return ld