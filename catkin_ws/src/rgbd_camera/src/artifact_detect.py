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
from ssd import build_ssd

class SPARSE2DENSE():
	def __init__(self):
		#rospy.loginfo("[%s] Initializing " %(self.node_name))
		self.bridge = CvBridge()
		self.cuda = True if torch.cuda.is_available() else False
		self.width = 640
		self.height = 480
		self.prob_threshold = 0.55
		self.cv_image = None
		self.border = 5
		self.net = build_ssd('test', 300, 2)    # initialize SSD
		self.net.load_weights('/home/arg_ws3/ssd.pytorch/weights/subt_real/subt_real_290.pth')
		if torch.cuda.is_available():
			self.net = self.net.cuda()

		self.generator = GeneratorUNet(in_channels=1, out_channels=1)
		if self.cuda:
			self.generator = self.generator.cuda()
		self.generator.load_state_dict(torch.load('/home/arg_ws3/PyTorch-GAN/implementations/pix2pix/saved_models/bbx_mask/generator_20.pth'))
		self.cv_depthimage = None
		self.generate_img = None
		self.Tensor = torch.cuda.FloatTensor if self.cuda else torch.FloatTensor
		self.data_transform = transforms.Compose([transforms.Resize((256, 256), PIL.Image.BICUBIC),
												  transforms.ToTensor()])
		#-------point cloud without color-------
		#self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.img_cb, queue_size = 1, buff_size = 2**24)
		#self.depth_sub = rospy.Subscriber("/X1/rgbd_camera/depth/image_raw", Image, self.img_cb, queue_size = 1, buff_size = 2**24)
		#------------------------------------

		#-------point cloud with color-------
		self.depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
		self.image_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
		self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 10, 10)
		self.ts.registerCallback(self.img_cb)
		#------------------------------------

		#self.pc_pub = rospy.Publisher("/pointcloud2_transformed", PointCloud2, queue_size=1)
		self.image_pub = rospy.Publisher("/generate_dp", Image, queue_size = 1)
		self.points = []
		rospy.loginfo("Start Generating depth image")

	def img_cb(self, rgb_data, depth_data):
		self.cv_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
		cv_depthimage = self.bridge.imgmsg_to_cv2(depth_data, "16UC1")
		self.predict(cv_depthimage)
		#self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.generate_img, "8UC1"))

	def predict(self, image):
		# Preprocessing
		#image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
		self.generate_img = np.zeros(image.shape, np.uint8)
		self.generate_img = self.cv_image.copy()
		image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
		image = image/10.
		x = cv2.resize(image, (300, 300)).astype(np.float32)
		x -= (104.0, 117.0, 123.0)
		x = x.astype(np.float32)
		x = x[:, :, ::-1].copy()
		x = torch.from_numpy(x).permute(2, 0, 1)

		#SSD Forward Pass
		xx = Variable(x.unsqueeze(0))     # wrap tensor in Variable
		if torch.cuda.is_available():
			xx = xx.cuda()
		y = self.net(xx)
		scale = torch.Tensor(image.shape[1::-1]).repeat(2)
		detections = y.data	# torch.Size([1, 4, 200, 5]) --> [batch?, class, object, coordinates]
		objs = []
		for i in range(detections.size(1)): # detections.size(1) --> class size
			for j in range(5):	# each class choose top 5 predictions
				if detections[0, i, j, 0].numpy() > self.prob_threshold:
					score = detections[0, i, j, 0]
					pt = (detections[0, i, j,1:]*scale).cpu().numpy()
					objs.append([pt[0], pt[1], pt[2]-pt[0]+1, pt[3]-pt[1]+1, i])
		for obj in objs:
			region = [int(obj[1] - self.border), int(obj[1] + obj[3] + self.border),\
					  int(obj[0] - self.border), int(obj[0] + obj[2] + self.border)]
			bbx_img = image[region[0] : region[1], region[2]: region[3]]
			generate_patch = self.generate_image(bbx_img)
			#cv2.resize(generate_patch, (obj[2], obj[3]))
			#cv2.imwrite('test.jpg', bbx_img)
			mask = generate_patch
			#ret, mask = cv2.threshold(generate_patch, 100, 255, cv2.THRESH_BINARY)
			mask = cv2.resize(mask, (region[3]-region[2], region[1]-region[0]))
			#self.generate_img[region[0] : region[1], region[2]: region[3]] = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
			cv2.rectangle(self.generate_img, (int(obj[0]), int(obj[1])),\
							(int(obj[0] + obj[2]), int(obj[1] + obj[3])), (255, 255, 0), 3)
		self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.generate_img, "8UC3"))
		#return img

	def generate_image(self, bbx_img):
		bbx_img = bbx_img/1000.
		#print(bbx_img.dtype)
		#print(bbx_img.shape)
		bbx_img = bbx_img[:,:,0]
		#print(bbx_img.shape)
		pil_im = PIL.Image.fromarray(bbx_img)
		pil_im = self.data_transform(pil_im)
		pil_im = pil_im.unsqueeze(0)

		my_img = Variable(pil_im.type(self.Tensor))
		my_img_fake = self.generator(my_img)
		my_img_fake = my_img_fake.squeeze(0).detach().cpu()
		pil_ = my_img_fake.mul(255).clamp(0, 255).byte().permute(1, 2, 0)
		pil_ = np.array(pil_)
		pil_ = pil_[...,::-1]
		#self.generate_img = cv2.resize(self.generate_img, (640, 480))
		#self.mask_dilate()
		#print("Hz: ", 1./(time.time() - prev_time))
		return pil_

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