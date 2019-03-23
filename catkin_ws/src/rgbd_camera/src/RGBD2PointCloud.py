#!/usr/bin/env python
import numpy as np
import cv2
import roslib
import rospy
import tf
import struct
import math
import time
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2, PointField
from geometry_msgs.msg import PoseArray, PoseStamped, Point
import rospkg
from nav_msgs.msg import Path
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
import message_filters

class RGBD2PointCloud():
	def __init__(self):
		#rospy.loginfo("[%s] Initializing " %(self.node_name))
		self.bridge = CvBridge()
		#msg = rospy.wait_for_message('/X1/rgbd_camera/rgb/camera_info', CameraInfo, timeout=None)
		msg = rospy.wait_for_message('/camera/color/camera_info', CameraInfo, timeout=None)
		self.fx = msg.P[0]
		self.fy = msg.P[5]
		self.cx = msg.P[2]
		self.cy = msg.P[6]

		#-------point cloud without color-------
		#self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.img_cb, queue_size = 1, buff_size = 2**24)
		#self.depth_sub = rospy.Subscriber("/X1/rgbd_camera/depth/image_raw", Image, self.img_cb, queue_size = 1, buff_size = 2**24)
		#------------------------------------

		#-------point cloud with color-------
		self.depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
		self.image_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
		self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 1, 0.3)
		self.ts.registerCallback(self.img_cb)
		#------------------------------------

		self.pc_pub = rospy.Publisher("/pointcloud2_transformed", PointCloud2, queue_size=1)
		self.points = []
	#-------point cloud without color-------
	# def img_cb(self, depth_data):
	# 	cv_depthimage = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
	# 	cv_depthimage2 = np.array(cv_depthimage, dtype=np.float32) 
	# 	print cv_depthimage2.shape
	# 	for x in range(cv_depthimage2.shape[0]):
	# 		for y in range(cv_depthimage2.shape[1]):
	# 			zc = cv_depthimage2[x][y]
	# 			print zc
	# 			position = self.getXYZ(y, x, zc/1000)
	# 			print position
	# 			pt = [position[0],position[1],position[2],0]
	# 			self.points.append(pt)
	# 	header = Header()
	# 	header.stamp = rospy.Time.now()
	# 	header.frame_id ="camera_link"
	# 	#header.frame_id = "X1/rgbd_camera_frame_optical"
	# 	fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1), PointField('z', 8, PointField.FLOAT32, 1), PointField('intensity', 16, PointField.FLOAT32, 1)]
	# 	pointcloud = point_cloud2.create_cloud(header, fields, self.points)
	# 	self.pc_pub.publish(pointcloud)
	# 	self.points = []
	#-------point cloud with color-------
	def img_cb(self, rgb_data, depth_data):
		cv_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
		print cv_image.shape
		cv_depthimage = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
		cv_depthimage2 = np.array(cv_depthimage, dtype=np.float32) 
		print cv_depthimage2.shape
		for x in range(cv_depthimage2.shape[0]):
			for y in range(cv_depthimage2.shape[1]):
				zc = cv_depthimage2[x][y]
				#print zc
				position = self.getXYZ(y, x, zc*0.001000)
				#print position

				b =	cv_image[x][y][0]
				g = cv_image[x][y][1]
				r = cv_image[x][y][2]
				a = 255
				rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
				pt = [position[0],position[1],position[2],rgb]
				self.points.append(pt)
		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id ="camera_color_optical_frame"
		#header.frame_id = "X1/rgbd_camera_frame_optical"
		fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1), PointField('z', 8, PointField.FLOAT32, 1), PointField('rgb', 12, PointField.UINT32, 1)]
		pointcloud = point_cloud2.create_cloud(header, fields, self.points)
		self.pc_pub.publish(pointcloud)
		self.points = []



	def getXYZ(self,xp, yp, zc):
	    #### Definition:
	    # cx, cy : image center(pixel)
	    # fx, fy : focal length
	    # xp, yp: index of the depth image
	    # zc: depth
	    inv_fx = 1.0/self.fx
	    inv_fy = 1.0/self.fy
	    x = (xp-self.cx) *  zc * inv_fx
	    y = (yp-self.cy) *  zc * inv_fy
	    z = zc
	    return (x,y,z)

if __name__ == '__main__':
	rospy.init_node('RGBD2PointCloud')
	foo = RGBD2PointCloud()
	rospy.spin()