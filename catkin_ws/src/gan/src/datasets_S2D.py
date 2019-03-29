import glob
import random
import os
import numpy as np

import torch
from torch.utils.data import Dataset
from PIL import Image
import torchvision.transforms as transforms
import cv2

class ImageDataset(Dataset):
    def __init__(self, root, transforms_=None, A_folder = 'depth', B_folder = 'pcl_32', mode='train'):
        self.transform = transforms.Compose(transforms_)
        self.root = root
        self.files = []
        self.A_folder = A_folder
        self.B_folder = B_folder
        for line in open(os.path.join(root, 'data_list', mode + '.txt')):
            self.files.append(line.strip())

    def __getitem__(self, index):
        idx = index % len(self.files)
        A_path = os.path.join(self.root, self.A_folder, self.files[idx] + ".png")
        B_path = os.path.join(self.root, self.B_folder, self.files[idx] + ".png")
        img_A = Image.open(A_path)
        img_B = Image.open(B_path)
        img_A = np.array(img_A)/655
        img_B = np.array(img_B)/655.

        #img_A = cv2.resize(img_A, (512, 512))
        #img_B = cv2.resize(img_B, (512, 512))

        #img_A = Image.fromarray(img_A)
        #img_B = Image.fromarray(img_B)

        img_A = torch.tensor(img_A).unsqueeze(dim=0)
        img_B = torch.tensor(img_B).unsqueeze(dim=0)

        #if np.random.random() < 0.5:
        #    img_A = Image.fromarray(np.array(img_A)[:, ::-1, :], 'RGB')
        #    img_B = Image.fromarray(np.array(img_B)[:, ::-1, :], 'RGB')

        # img_A = self.transform(img_A)
        # img_B = self.transform(img_B)

        return {'A': img_A, 'B': img_B}

    def __len__(self):
        return len(self.files)
