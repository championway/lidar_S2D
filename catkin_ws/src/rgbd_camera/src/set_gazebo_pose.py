#!/usr/bin/env python
import numpy as np
import cv2
import roslib
import rospy
import tf
import math
import time
import random
import rospkg
from std_msgs.msg import Header
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest

class SetGAZEBO():
	def __init__(self):
		#-------point cloud without color-------
		rospy.loginfo("Start to set gazebo pose")		
		self.set_gazebo_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
		self.save_image_srv = rospy.ServiceProxy("/save_image", Trigger)
		self.state = ModelState()
		self.robot_state = ModelState()
		self.set_model_state = SetModelState()
		self.theta_range = (-28, 28)
		self.distance_range = (2, 25)
		self.yaw_range = (0, 360)
		self.counter = 0
		self.object_list = ['euro_pallet', 'euro_pallet_0', 'first_2015_trash_can', 'follower_vehicle', 'Mailbox']
		while(True):
			self.set_gazebo_pose()
			rospy.sleep(2)
			self.save_image()
			rospy.sleep(1.5)

	def set_gazebo_pose(self):
		try:
			position_list = []
			rospy.wait_for_service("/gazebo/set_model_state")
			self.robot_state.model_name = "bot"
			self.robot_state.pose.position.x = 0
			self.robot_state.pose.position.y = 0
			self.robot_state.pose.position.z = 0
			rad = self.set_gazebo_srv(self.robot_state)

			for obj in self.object_list:
				self.state.model_name = obj
				too_close = True
				p = None
				while too_close:
					too_close = False
					degree, r, quat = self.get_pose(self.theta_range, self.distance_range, self.yaw_range)
					p = [r * np.cos(np.radians(degree)), r * np.sin(np.radians(degree))]
					for pos in position_list:
						if self.distanse(p, pos) < 3:
							too_close = True
				position_list.append(p)
				self.set_pose(p, quat)
				ret = self.set_gazebo_srv(self.state)

			rospy.loginfo("Send service: " + str(self.counter))
			self.counter = self.counter + 1

		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))

	def set_pose(self, p, q):
		self.state.pose.position.x = p[0]
		self.state.pose.position.y = p[1]
		self.state.pose.position.z = 0
		self.state.pose.orientation.x = q[0]
		self.state.pose.orientation.y = q[1]
		self.state.pose.orientation.z = q[2]
		self.state.pose.orientation.w = q[3]

	def get_pose(self, theta_range, distance_range, yaw_range):
		degree = random.randint(theta_range[0]*100, theta_range[1]*100)/100.
		r = random.randint(distance_range[0]*100, distance_range[1]*100)/100.
		yaw = random.randint(yaw_range[0]*100, yaw_range[1]*100)/100.
		quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
		return degree, r, quat

	def save_image(self):
		try:
			rospy.wait_for_service("/save_image")
			self.save_image_srv()
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))

	def distanse(self, p1, p2):
		return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


if __name__ == '__main__':
	rospy.init_node('SetGAZEBO')
	foo = SetGAZEBO()
	rospy.spin()