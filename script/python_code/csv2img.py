import os
import cv2
import numpy as np
from matplotlib.pyplot import figure
import matplotlib.pyplot as plt
from matplotlib.image import imread
from numpy.core._multiarray_umath import ndarray

from utils import colored_depthmap
from convert_str2nparray import convert_img2csv_str2nparray, convert_depth2csv_str2nparray

rel_path = '/home/hj/ICRA2020/190403/depth2csv/'
img_rel_path = '/home/hj/ICRA2020/190403/img/'
save_rel_path = '/home/hj/ICRA2020/debug/depth2csv/'


def save_colored_depth(rel_path):

    fList = os.listdir(rel_path)
    NewList = sorted(fList, key=lambda x: int(x[7:-4]))

    count = 0

    for file in NewList:
        depth_path = rel_path + file
        depth_array = convert_img2csv_str2nparray(depth_path)
        new_depth_image = colored_depthmap(depth_array)

        save_path = save_rel_path + file[:-3] +'png'
        cv2.imwrite(save_path, new_depth_image)

        count = count + 1
        #if count == 30:
            #break


def plot_colored_depth(rel_path):

    fList = os.listdir(rel_path)
    NewList = sorted(fList, key=lambda x: int(x[7:-4]))

    count = 0

    for file in NewList:
        depth_path = rel_path + file
        depth_array = convert_img2csv_str2nparray(depth_path)

        new_depth_image = colored_depthmap(depth_array)


        '''
        Use the following
        if you want to plot rgb and depth concurrently: 
        
        img_path = img_rel_path + file[:-3] + 'png'
        img = imread(img_path)
        f = plt.figure()
        f.set_size_inches(18.5, 10.5, forward=True)
        f.add_subplot(1, 2, 1)
        plt.imshow(new_depth_image, cmap='jet')
        f.add_subplot(1, 2, 2)
        plt.imshow(img)
        plt.show()
        plt.clf()
        plt.close()
        '''

        cv2.imshow("colored depth", new_depth_image)
        cv2.waitKey(0)

        count = count + 1
        if count == 30:
            break


def show_depth_csv(rel_path):

    fList = os.listdir(rel_path)
    NewList = sorted(fList, key=lambda x: int(x[7:-4]))

    count = 0

    for file in NewList:
        depth_path = rel_path + file
        depth_array = convert_depth2csv_str2nparray(depth_path)
        print(depth_array)

        count = count + 1
        if count == 1:
            break


if __name__ == "__main__":
    #save_colored_depth(rel_path)
    #plot_colored_depth(rel_path)
    show_depth_csv(rel_path)