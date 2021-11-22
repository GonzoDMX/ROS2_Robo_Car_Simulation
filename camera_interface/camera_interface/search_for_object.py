
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_parameters
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan

from obj_tracking_msgs.msg import ObjTracker
from obj_tracking_msgs.msg import SetTracker



class ObjectSearch(Node):

	def __init__(self):
		super().__init__('object_search_routine')
		self.subscription = self.create_subscription(
			ObjTracker,
			'/robo_car/track_object',
			self.listener_callback,
			qos_profile_sensor_data)
			
		self.recv_goal = self.create_subscription(
			SetTracker,
			'/robo_car/init_track',
			self.goal_callback,
			qos_profile_sensor_data)
			
		self.pub_move = self.create_publisher(
			Twist,
			'/robo_car/cmd_vel',
			qos_profile_parameters)

		self.imu_sub = self.create_subscription(
			Imu,
			'/robo_car/imu',
			self.imu_callback,
			qos_profile_sensor_data)		

		self.laser_sub = self.create_subscription(
			LaserScan,
			'robo_car/laser_scan',
			self.laser_callback,
			qos_profile_sensor_data)
			

		self.curr_orient = 0.0
		self.start_orient = 0.0
		self.spin_check = False
		self.distance = 10.0
		print("Object tracker node online")
		print("Waiting for request")		
		self.status = 0
		self.publish_move(0.0, 0.0)

		
	def listener_callback(self, msg):
		# Search current location
		if self.status == 1:
			if msg.count > 0:
				m = round(msg.x_position[0], 2) * -0.5
				self.publish_move(0.0, m)
				if m == 0:
					print("Object Found !")
					self.status = 2
					print("Moving to object")
			elif self.spin_check and self.curr_orient == self.start_orient:
				print("No objects found in local vicinity")
				self.status = 3
				self.publish_move(0.0, 0.0)
				print("Entering search pattern")
			if self.curr_orient != self.start_orient and not self.spin_check:
				self.spin_check = True
		# Approach Object
		if self.status == 2:
			if msg.count > 0:
				y = round(msg.y_position[0], 2)
				x = round(msg.x_position[0], 2) * -0.5
				self.publish_move(y, x)
				if y == 0:
					print("I've arrived at the object")
					print("Tracking sequence complete!")
					self.status = 0
		# Search another location
		if self.status == 3:
			if self.distance < 1:
				self.publish_move(0.0, 0.3)
			else:
				self.publish_move(0.7, 0.0)
			if msg.count > 0:
				self.publish_move(0.0, 0.0)
				self.status = 1
			

	def goal_callback(self, g):
		if g.goal == 1:
			self.status = g.goal
			print("Starting object search routine.")
			self.publish_move(0.0, 0.2)
		if g.goal == -1:
			self.status = 0
			self.publish_move(0.0, 0.0)
			print("Object tracking terminated")
			print()
			print("Awaiting next request")

	def imu_callback(self, msg):
		self.curr_orient = round(msg.orientation.z, 2)


	def laser_callback(self, msg):
		self.distance = min(msg.ranges)

	def publish_move(self, vel_x, vel_z):
		msg = Twist()
		msg.linear.x = vel_x
		msg.angular.z = vel_z
		self.pub_move.publish(msg)
		

def main(args=None):
	rclpy.init(args=args)

	obj_searcher = ObjectSearch()

	rclpy.spin(obj_searcher)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	cam_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
