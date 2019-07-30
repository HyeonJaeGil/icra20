import os
import cv2
import numpy as np
from matplotlib.pyplot import figure
import matplotlib.pyplot as plt
from matplotlib.image import imread
from utils import colored_depthmap
from convert_str2nparray import convert_img2csv_str2nparray

rel_path = '/home/hj/ICRA2020/190403/rp_img2csv/'
img_rel_path = '/home/hj/ICRA2020/190403/img/'
save_rel_path = '/home/hj/ICRA2020/190403/debug/'

fList = os.listdir(rel_path)
NewList = sorted(fList, key=lambda x: int(x[7:-4]))

count = 0

for file in NewList:
    depth_path = rel_path + file
    NpArray = convert_img2csv_str2nparray(depth_path)
    CMap = colored_depthmap(NpArray, 0)

    img_path = img_rel_path + file[:-3] + 'png'
    img = imread(img_path)

    save_path = save_rel_path + file[:-3] +'png'

    f = plt.figure()
    f.set_size_inches(18.5, 10.5, forward=True)
    f.add_subplot(1, 2, 1)
    plt.imshow(CMap, 'bwr', origin='lower')
    f.add_subplot(1, 2, 2)
    plt.imshow(img)
    plt.show()

    #f.savefig(save_path)

    plt.clf()
    plt.close()
    count = count + 1
    #if count == 4:
        #break



#array = convert_img2csv_str2nparray('/home/hj/ICRA2020/190403/rp_img2csv')
