import h5py
import os
import numpy as np
import cv2
import matplotlib.pyplot as plt
import open3d as o3d

from utils import colored_depthmap
from fill_depth2 import fill_depth_colorization


def visualize_h5f(h5_path):
    '''
    :param h5_path:
    :return: void
    visualize bgr + colorized depth + colorized LiDAR
    '''
    h5f = h5py.File(h5_path, "r")
    rgb_data = np.array(h5f['rgb'], dtype=np.uint8)
    # Originally, shape of rgb_data: (3, 480, 640)
    rgb = np.transpose(rgb_data, (1, 2, 0))
    bgr = rgb[:, :, ::-1]

    depth = np.array(h5f['depth'], dtype=np.float32)
    max_depth = np.amax(depth)
    min_depth = np.amin(depth)
    colored_depth = np.array(colored_depthmap(depth, min_depth, max_depth), dtype=np.uint8)
    colored_depth = colored_depth[:, :, ::-1]

    projected = np.array(h5f['scan_projected'], dtype=np.float32)
    colored_pc = np.array(colored_depthmap(projected, min_depth, max_depth), dtype=np.uint8)
    colored_pc = colored_pc[:, :, ::-1]
    im_result = cv2.hconcat([bgr, colored_depth, colored_pc])
    cv2.imshow("visualized", im_result)
    cv2.waitKey(0)
    # cv2.destroyAllWindows()


def visualize_depth_and_projected_img(img_path, h5_path):
    '''
     :param h5_path, LiDAR-projected RGB img_path:
     :return: void
     visualize bgr with colorized LiDAR projected + colorized depth
     '''
    h5f = h5py.File(h5_path, "r")
    depth = np.array(h5f['depth'], dtype=np.float32)
    #print("Shape of depth: ", depth.shape)
    max_depth = np.amax(depth)
    min_depth = np.amin(depth)
    colored_depth = np.array(colored_depthmap(depth, min_depth, max_depth), dtype=np.uint8)
    #print("Shape of colored_depth: ", colored_depth.shape)
    colored_depth = colored_depth[:, :, ::-1]

    projected_img = cv2.imread(img_path, cv2.IMREAD_COLOR)
    print(projected_img.shape)

    im_result = cv2.hconcat([projected_img, colored_depth])
    cv2.imshow("visualized", im_result)
    cv2.waitKey(0)
    # cv2.destroyAllWindows()


if __name__ == "__main__":

    ## variable to change! ##
    dir_name = 'for_pcl'

    #for hj laptop - external SSD
    h5_file_path = '/media/hj/Samsung_T5/icra2020/'+ dir_name + '/h5'

    fList = os.listdir(h5_file_path)
    new_list = sorted(fList, key=lambda x: int(x[:-3]))
    print(new_list)

    count = 0

    for csv_file in new_list:

        print(csv_file)
        print(os.path.join(h5_file_path, csv_file))

        visualize_depth_and_projected_img(projected_img_save_path + '/' + csv_file[:-3] + '.png', os.path.join(h5_file_path, csv_file))
        visualize_h5f(os.path.join(h5_file_path, csv_file))

        count += 1

        # if count == 1:
        #     break
