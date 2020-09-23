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
		    {'interface_type': 'usb'},
		    {"serial_port": "/dev/ttyUSB0"},
		    {'usb_selector': "index"},
		    {'usb_index': 0},
		    {'usb_serial_number': "00000000"},
		    {'usb_description': "Robotic Mobile Platform"},
		    {'motor_timeout': 0.5},
		    {'frame_id': "base_link"},
		    {'odom_frame_id': "odom"},
		    {'invert_linear_vel_cmds': 'false'},
		    {'invert_angular_vel_cmds': 'false'},
		    {'broadcast_tf': 'true'},
		    {'rmp_type': "200/400"},
		    {'linear_pos_accel_limit': 0.0},
		    {'linear_neg_accel_limit': 0.0},
		    {'angular_pos_accel_limit': 0.0},
		    {'angular_neg_accel_limit': 0.0},
		    {'max_linear_vel': 0.0},
		    {'max_angular_vel': 0.},
		    {'linear_odom_scale': 1.0},
		    {'angular_odom_scale': 1.0},
		    {'reset_odometry': 'false'},
		    {'odometry_reset_duration': 1.0}
            ]
        )
    ])
