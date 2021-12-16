from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():

	declare_use_t265_argument = DeclareLaunchArgument(
		'use_t265',
		default_value='True',
		description='')
	
	t265_root_node = Node(
		package='tf2_ros',
		executable = 'static_transform_publisher',
		arguments = ["-0.08", "0.01", "-0.115", "0", "0", "0", "camera_pose", "base_footprint"],
		condition=IfCondition(LaunchConfiguration("use_t265")),
		output='screen')
	
	d435_branch_node = Node(
		package='tf2_ros',
		executable = 'static_transform_publisher',
		arguments = ["-0.0489", "0.0175", "0.139", "0", "0", "0", "base_footprint", "camera_link"],
		condition=IfCondition(LaunchConfiguration("use_t265")),
		output='screen')
	
	tf_pub_node = Node(
		package='odom_tf_pub',
		executable='odom_tf_pub',
		parameters=[{'select_t265_odom': LaunchConfiguration("use_t265")}],
		output='screen')
	
	return LaunchDescription([
		declare_use_t265_argument,
		t265_root_node,
		d435_branch_node,
		tf_pub_node
		])


