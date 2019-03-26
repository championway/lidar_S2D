import argparse
import os
import numpy as np
import math
import itertools
import time
import datetime
import sys
import cv2

import torchvision.transforms as transforms
from torchvision.utils import save_image

from torch.utils.data import DataLoader
from torchvision import datasets
from torch.autograd import Variable

from models import *
from datasets_S2D import *

import torch.nn as nn
import torch.nn.functional as F
import torch

model_epoch = 24

parser = argparse.ArgumentParser()
parser.add_argument('--img_height', type=int, default=256, help='size of image height')
parser.add_argument('--img_width', type=int, default=256, help='size of image width')
parser.add_argument('--channels', type=int, default=1, help='number of image channels')
opt = parser.parse_args()
print(opt)

cuda = True if torch.cuda.is_available() else False

# Initialize generator and discriminator
generator = GeneratorUNet(in_channels=1, out_channels=1)
discriminator = Discriminator(in_channels=1)

if cuda:
    generator = generator.cuda()
    discriminator = discriminator.cuda()

generator.load_state_dict(torch.load('/media/arg_ws3/5E703E3A703E18EB/data/lidar_S2D/result/saved_models/S2D/generator_%d.pth'%(model_epoch)))
#discriminator.load_state_dict(torch.load('saved_models/sparse2dense/discriminator_%d.pth'%(model_epoch)))

# Configure dataloaders
transforms_A = [ transforms.Resize((opt.img_height, opt.img_width), Image.BICUBIC),
                transforms.ToTensor(),
                transforms.Normalize((0.5,0.5,0.5), (0.5,0.5,0.5)) ]
transforms_B = [ transforms.Resize((opt.img_height, opt.img_width), Image.BICUBIC),
                transforms.ToTensor() ]

data_transform = transforms.Compose(transforms_B)

# Tensor type
Tensor = torch.cuda.FloatTensor if cuda else torch.FloatTensor

def sample_images():
    """Saves a generated sample from the validation set"""
    prev_time = time.time()
    image = cv2.imread("/media/arg_ws3/5E703E3A703E18EB/data/lidar_S2D/pcl_32/img_2.png",cv2.IMREAD_ANYDEPTH)
    
    image = np.array(image)/655.
    image = cv2.resize(image, (256, 256))
    image = torch.tensor(image).unsqueeze(dim=0).unsqueeze(dim=0)
    image = Variable(image.type(Tensor))
    print(image.shape)
    # print(image.dtype)
    # pil_im = Image.fromarray(image)
    # pil_im = data_transform(pil_im)
    # pil_im = pil_im.unsqueeze(0)

    # my_img = Variable(pil_im.type(Tensor))
    my_img_fake = generator(image)
    my_img_fake = my_img_fake.squeeze(0).detach().cpu()

    # pil_ = my_img_fake.mul(100).clamp(0, 100).to(torch.int16).permute(1, 2, 0)
    pil = my_img_fake.to(torch.int16).permute(1, 2, 0)
    pil = np.array(pil)
    pil = pil[...,::-1]
    pil = cv2.resize(pil, (640, 480))
    print(pil.dtype)
    # print(pil_.dtype)
    # print(pil_)
    cv2.imwrite("dep.png", pil)
    print("Hz: ", 1./(time.time() - prev_time))
    save_image(my_img_fake.data, 'tt.png', nrow=1, normalize=False)
    #print(my_img_fake.data)

sample_images()