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
from s2d_msgs.msg import S2D_Image, S2D_ImageList
import os
import math
import time
import sys
import PIL
from models import *
import torchvision.transforms as transforms
from torch.autograd import Variable
#from models import *
import torch.nn as nn
import torch.nn.functional as F
import torch

class SPARSE2DENSE():
	def __init__(self):
		#rospy.loginfo("[%s] Initializing " %(self.node_name))
		self.bridge = CvBridge()
		self.cuda = True if torch.cuda.is_available() else False
		self.generator = GeneratorUNet(in_channels=1, out_channels=1)
		if self.cuda:
			self.generator = self.generator.cuda()
		self.generator.load_state_dict(torch.load('/media/arg_ws3/5E703E3A703E18EB/data/lidar_S2D/result/saved_models/S2D/generator.pth'))
		self.cv_depthimage = None
		self.generate_img = None
		self.Tensor = torch.cuda.FloatTensor if self.cuda else torch.FloatTensor
		self.data_transform = transforms.Compose([transforms.Resize((512, 512), 
												  PIL.Image.BICUBIC),
												  transforms.ToTensor()])
		#-------point cloud without color-------
		self.data_sub = rospy.Subscriber("/l2d_data", S2D_ImageList, self.data_cb, queue_size = 1, buff_size = 2**24)
		#self.depth_sub = rospy.Subscriber("/X1/rgbd_camera/depth/image_raw", Image, self.img_cb, queue_size = 1, buff_size = 2**24)
		#------------------------------------

		#-------point cloud with color-------
		#self.depth_sub = message_filters.Subscriber("/X1/rgbd_camera/depth/image_raw", Image)
		#self.image_sub = message_filters.Subscriber("/X1/rgbd_camera/rgb/image_raw", Image)
		#self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 1, 0.3)
		#self.ts.registerCallback(self.img_cb)
		#------------------------------------

		#self.pc_pub = rospy.Publisher("/pointcloud2_transformed", PointCloud2, queue_size=1)
		self.image_pub = rospy.Publisher("/gan_img", Image, queue_size = 1)
		self.data_pub = rospy.Publisher("/gan_data", S2D_ImageList, queue_size = 1)
		self.points = []
		rospy.loginfo("Start Generating depth image")
		# self.cv_depthimage = cv2.imread('/media/arg_ws3/5E703E3A703E18EB/data/lidar_S2D/depth_655/img_366.png', cv2.IMREAD_ANYDEPTH)
		# self.cv_depthimage = self.cv_depthimage.astype(np.int16)
		# while(True):
		# 	#self.generate_image()
		# 	self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_depthimage, "16SC1"))
		# 	rospy.sleep(0.5)

	def data_cb(self, msg):
		s2d_data = S2D_ImageList()
		prev_time = time.time()
		for i in range(msg.size):
			s2d_img = S2D_Image()
			s2d_img = msg.list[i]
			self.cv_depthimage = self.bridge.imgmsg_to_cv2(msg.list[i].img, "16UC1")
			self.generate_image()
			s2d_img.img = self.bridge.cv2_to_imgmsg(self.generate_img, "16UC1")
			s2d_data.list.append(s2d_img)
		s2d_data.size = len(s2d_data.list)
		self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.generate_img, "16UC1"))
		self.data_pub.publish(s2d_data)
		print("Hz: ", 1./(time.time() - prev_time))

	def generate_image(self):
		prev_time = time.time()
		# self.cv_depthimage = np.array(self.cv_depthimage)/10000.

		image = np.array(self.cv_depthimage)
		image = image/655.
		## image = cv2.resize(image, (512, 512))
		image = torch.tensor(image).unsqueeze(dim=0).unsqueeze(dim=0)
		image = Variable(image.type(self.Tensor))

		# pil_im = PIL.Image.fromarray(self.cv_depthimage)
		# pil_im = self.data_transform(pil_im)
		# pil_im = pil_im.unsqueeze(0)

		# my_img = Variable(pil_im.type(self.Tensor))
		my_img_fake = self.generator(image)
		my_img_fake = my_img_fake.squeeze(0).detach().cpu()
		#pil_ = my_img_fake.mul(255).clamp(0, 255).to(torch.float32).permute(1, 2, 0)
		# pil_ = my_img_fake.mul(10000).clamp(0, 10000).to(torch.int16).permute(1, 2, 0)
		# #pil_ = my_img_fake.mul(255).clamp(0, 255).byte().permute(1, 2, 0)
		# pil_ = np.array(pil_)
		# self.generate_img = pil_[...,::-1]
		# #pil_ = np.float32(pil_)
		# #self.mask_dilate()
		# self.generate_img = cv2.resize(self.generate_img, (640, 480))
		pil = my_img_fake.permute(1, 2, 0)
		pil = np.array(pil)
		pil = pil[...,::-1]
		## pil = cv2.resize(pil, (512, 512), interpolation=cv2.INTER_CUBIC)
		pil = pil*655.
		pil[pil < 0] = 0
		pil = pil.astype(np.uint16)
		#cv2.imwrite('sss.png', pil/655.)
		self.generate_img = pil
		#self.mask_dilate()
		#print("Hz: ", 1./(time.time() - prev_time))
	def mask_dilate(self):
		mask = np.zeros(self.generate_img.shape, np.uint8)
		ret, mask = cv2.threshold(self.generate_img, 500, 255, cv2.THRESH_BINARY)
		mask = 255 - mask
		kernel = np.ones((10,10),np.uint8)
		mask = cv2.dilate(mask, kernel, iterations = 1)
		mask = 255 - mask
		for j in range(mask.shape[0]):
			for i in range(mask.shape[1]):
				if mask[j][i] == 0:
					self.generate_img[j][i] = 0

if __name__ == '__main__':
	rospy.init_node('SPARSE2DENSE')
	foo = SPARSE2DENSE()
	rospy.spin()