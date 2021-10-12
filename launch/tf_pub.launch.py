import os
from pathlib import Path
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	#use_sim_time = LaunchConfiguration('use_sim_time')
	#slam_params_file = LaunchConfiguration('slam_params_file')
	#indoor_testing_dir = os.path.join(str(Path(__file__).parents[4]), 'src/indoor_testing')
	#nav2_bringup = get_package_share_directory('nav2_bringup')
	switch = True
	t_or_f = {True : 'true', False : 'false'}
	t265_odom = LaunchConfiguration('select_t265_odom', default=t_or_f[switch])

	declare_select_t265_odom_argument = DeclareLaunchArgument(
		'select_t265_odom',
		default_value='false',
		description='')
		
	print(f"!!!!!!!!! {t_or_f[switch]}")
	if switch:
		tf2_node = Node(
			package='tf2_ros',
			executable = 'static_transform_publisher',
			arguments = ["-0.08", "0", "-0.115", "0", "0", "0", "camera_pose", "base_footprint"])
		print("T265 static tf")
	else:
		tf2_node = Node(
			package='tf2_ros',
			executable = 'static_transform_publisher',
			arguments = ["-0.06", "-0.06", "-0.16", "0", "0", "0", "zed2_base_link", "base_footprint"])
		print("Zed2 static tf")
		
	tf_pub_node = Node(
		package='tf_pub',
		executable='tf_pub',
		parameters=[{'select_t265_odom': t265_odom}],
		output='screen')
		
	return LaunchDescription([
		declare_select_t265_odom_argument,
		tf2_node,
		tf_pub_node
		])


