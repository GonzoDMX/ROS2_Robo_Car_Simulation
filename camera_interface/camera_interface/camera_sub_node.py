
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import Image

from obj_tracking_msgs.msg import ObjTracker

class TrackCameraBroadcaster(Node):

	def __init__(self):
		super().__init__('track_cam_publisher')
		self.publisher_ = self.create_publisher(
			Image,
			'/robo_car/track_camera',
			qos_profile_sensor_data)

	def publishIm(self, msg):
		self.publisher_.publish(msg)

		
class ObjectTrackBroadcaster(Node):

	def __init__(self):
		super().__init__('obj_track_publisher')
		self.publisher_ = self.create_publisher(
			ObjTracker,
			'/robo_car/track_object',
			qos_profile_sensor_data)

	def publishMsg(self, msg):
		self.publisher_.publish(msg)
		

class CvCameraSubscriber(Node):

	def __init__(self):
		super().__init__('cv_camera_subscriber')
		self.br = CvBridge()
		self.cam_caster = TrackCameraBroadcaster()
		self.obj_caster = ObjectTrackBroadcaster()
		self.subscription = self.create_subscription(
			Image,
			'/camera/image_raw',
			self.listener_callback,
			qos_profile_sensor_data)
		self.subscription  # prevent unused variable warning

	def listener_callback(self, msg):
		# Get camera feed and convert to OpenCV Image
		im = self.br.imgmsg_to_cv2(msg, "bgr8")
		
		# Get image width
		im_width, im_height, _ = im.shape
		
		# Define color thresholds, we want bright red objects
		lower_red = np.array([0,0,120])
		upper_red = np.array([50,50,255]) 
		
		# Create a mask
		mask = cv.inRange(im, lower_red, upper_red)  
		# get all non zero values
		coord=cv.findNonZero(mask)
		# Create threshold from image (binary) mask
		thresh = cv.threshold(mask,128,255,cv.THRESH_BINARY)[1]

		# Set padding size for Bounding Box
		pad = 10
		# Set variable for counting found objects
		count = 0
		x_pos = []
		y_pos = []
		sizes = []
		# Find objects in mask by contour
		contours = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
		contours = contours[0] if len(contours) == 2 else contours[1]
		
		# For object contours found, for handling multiple objects
		for cntr in contours:
			count += 1
			# Get dimensions of object
			x,y,w,h = cv.boundingRect(cntr)
			sizes.append(x*y)
			# Create a bounding box
			cv.rectangle(im, (x-pad, y-pad), (x+w+pad, y+h+pad), (36, 255, 12), 2)
			# Label the bounding box
			cv.putText(im, 'Target ' + str(count), (x, y-20), cv.FONT_HERSHEY_SIMPLEX, 0.4, (36, 36, 255), 2)
			ctr_x = x+(w/2)
			ctr_y = y+(h/2)

			# Get x positional value for movement			
			normal_x = round(((ctr_x * 2) / im_width) + -1, 6)
			x_pos.append(normal_x)
			
			# Get y positional value for approach
			normal_y = round((im_height - (y + h)) / im_height, 6)
			y_pos.append(normal_y)
			
			# Determine object center point
			center = (int(ctr_x), int(ctr_y))
			# Mark object center point
			cv.circle(im, center, radius=1, color=(36, 255, 12), thickness=-1)

		message = ObjTracker()
		message.count = count
		message.x_position = x_pos
		message.y_position = y_pos
		message.size = sizes

		self.obj_caster.publishMsg(message)

		# Convert OpenCV Image back to ROS2 Image Msg
		msg.data = self.br.cv2_to_imgmsg(im, encoding="bgr8").data
		msg.encoding = 'bgr8'
		# Broadcast the Image feed with bounding boxes
		self.cam_caster.publishIm(msg)
		
		''' Display Image with Bounding Box and Image mask '''
		# cv.imshow("Bounded",im)
		# cv.imshow('Mask', mask)
		# cv.waitKey()


def main(args=None):
	rclpy.init(args=args)

	cam_subscriber = CvCameraSubscriber()

	rclpy.spin(cam_subscriber)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	cam_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
