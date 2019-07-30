import os
import cv2
from random import random
# import torch
import shutil
import numpy as np
import matplotlib.pyplot as plt
# from PIL import Image
from convert_str2nparray import convert_img2csv_str2nparray

# cmap = plt.cm.viridis
cmap = plt.cm.jet


def colored_depthmap(depth, d_min=None, d_max=None):
    if d_min is None:
        d_min = np.min(depth)
    if d_max is None:
        d_max = np.max(depth)
    depth_relative = (depth - d_min) / (d_max - d_min)
    # depth_relative = (d_max - depth) / (d_max - d_min)
    return 255 * cmap(depth_relative)[:, :, :3]  # H, W, C


if __name__ == "__main__":
    rel_path = '/home/hj/ICRA2020/190403/rp_img2csv/'
    f_list = os.listdir(rel_path)
    new_list = sorted(f_list, key=lambda x: int(x[7:-4]))
    for file in new_list:
        path = rel_path + file
        array_ = convert_img2csv_str2nparray(path)
        print(array_.shape)
        print(array_.dtype)
        # int_array = array.astype(np.int8)
        c_map = colored_depthmap(array_)
        int_c_map = c_map.astype(np.int8)
        # print(type(cmap))
        # print(cmap)
        # print(type(array))
        # print(array)
        plt.imshow(int_c_map, "seismic")
        plt.show()
