from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
	
	zed2_branch_node = Node(
		package='tf2_ros',
		executable = 'static_transform_publisher',
		arguments = ["0.022", "0.094", "0.147", "0.6336", "0", "0", "base_footprint", "zed2_l_base_link"],
		output='screen')
  
	zed2_r_branch_node = Node(
		package='tf2_ros',
		executable = 'static_transform_publisher',
		arguments = ["0.022", "-0.094", "0.147", "-0.6615", "0", "0", "base_footprint", "zed2_r_base_link"],
		output='screen')
		
	zed2_l_filter = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([get_package_share_directory('depth_filter'), '/launch/depth_filter.launch.py']),
		launch_arguments={'depth_sub_topic': '/zed2_l/zed_node/depth/depth_registered',
		'color_info_topic': '/zed2_l/zed_node/left/camera_info',
		'depth_pub_topic': '/zed2_l/zed_node/depth/depth_filtered',
		'max_height': '1.5'}.items(),
	)
	zed2_r_filter = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([get_package_share_directory('depth_filter'), '/launch/depth_filter.launch.py']),
		launch_arguments={'depth_sub_topic': '/zed2_r/zed_node/depth/depth_registered',
		'color_info_topic': '/zed2_r/zed_node/left/camera_info',
		'depth_pub_topic': '/zed2_r/zed_node/depth/depth_filtered',
		'max_height': '1.5'}.items(),
	)
	return LaunchDescription([
		zed2_branch_node,
		zed2_r_branch_node,
		zed2_l_filter,
		zed2_r_filter
		])
