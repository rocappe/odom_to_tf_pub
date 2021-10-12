#!/usr/bin/env/python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry


class TfPublisher(Node):

	def __init__(self):
		super().__init__('tf_publisher_node')
		print("Initializing tf_publisher_node")
		self.qos = QoSProfile(depth=10)
		self.tf_pub = self.create_publisher(
						TFMessage,
						'/tf',
						self.qos)
		self.declare_parameter('select_t265_odom')
		
		self.selector()
		
	def selector(self):
		if self.get_parameter('select_t265_odom').get_parameter_value().bool_value:
			self.t265_odom_sub = self.create_subscription(
						Odometry,
						'/t265/odom',
						self.odom_callback,
						self.qos)
			print("Subscribed to /t265/odom")
		else:
			self.zed2_odom_sub = self.create_subscription(
						Odometry,
						'/zed2/zed_node/odom',
						self.odom_callback,
						self.qos)
			print("Subscribed to /zed2/zed_node/odom")

	def odom_callback(self, msg):
		# Publish TF data
		tf_msg = TransformStamped()
		#tf_msg.header.stamp = Node.get_clock(self).now().to_msg()
		tf_msg.header.stamp = msg.header.stamp
		tf_msg.header.frame_id = msg.header.frame_id
		tf_msg.child_frame_id = msg.child_frame_id
		tf_msg.transform.translation.x = msg.pose.pose.position.x 
		tf_msg.transform.translation.y = msg.pose.pose.position.y 
		tf_msg.transform.translation.z = msg.pose.pose.position.z 
		tf_msg.transform.rotation.x = msg.pose.pose.orientation.x
		tf_msg.transform.rotation.y = msg.pose.pose.orientation.y
		tf_msg.transform.rotation.z = msg.pose.pose.orientation.z
		tf_msg.transform.rotation.w = msg.pose.pose.orientation.w
		message = TFMessage()
		message.transforms = [tf_msg]
		self.tf_pub.publish(message)
		

def main(args=None):
	rclpy.init()
	tf_publisher_node = TfPublisher()

	try:
		rclpy.spin(tf_publisher_node)
	except KeyboardInterrupt:
		pass

	tf_publisher_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

