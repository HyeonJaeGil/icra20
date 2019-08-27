import h5py
import os
import numpy as np
import cv2
import matplotlib.pyplot as plt
import open3d as o3d

from utils import colored_depthmap
from fill_depth2 import fill_depth_colorization


def save_concated_img(h5_path, file, save_path):
    '''
     :param h5_path, file, save_path:
     :return: void
     save concated bgr + colorized depth + colorized LiDAR as .png
     '''
    h5f = h5py.File(h5_path, "r")
    rgb_data = np.array(h5f['rgb'], dtype=np.uint8)
    # Originally, shape of rgb_data: (3, 480, 640)
    rgb = np.transpose(rgb_data, (1, 2, 0))
    bgr = rgb[:, :, ::-1]

    depth = np.array(h5f['depth'], dtype=np.float32)
    max_depth = np.amax(depth)
    min_depth = np.amin(depth)
    # max_depth = 10
    # min_depth = 0
    colored_depth = np.array(colored_depthmap(depth, min_depth, max_depth), dtype=np.uint8)
    colored_depth = colored_depth[:, :, ::-1]

    projected = np.array(h5f['scan_projected'], dtype=np.float32)
    # max_depth = np.amax(projected)
    # min_depth = np.amin(projected)
    colored_pc = np.array(colored_depthmap(projected, min_depth, max_depth), dtype=np.uint8)
    colored_pc = colored_pc[:, :, ::-1]
    im_result = cv2.hconcat([bgr, colored_depth, colored_pc])
    save_file_path = os.path.join(save_path, file[:-3]) + '.png'
    print(save_file_path)
    cv2.imwrite(save_file_path, im_result)


def save_depth_filled_csv(h5_path, file, save_path):
    '''
     :param h5_path, file, save_path:
     :return: void
     save new_depth2csv file (depth-filled)
     '''
    h5f = h5py.File(h5_path, "r")
    rgb_data = np.array(h5f['rgb'], dtype=np.uint8)
    # Originally, shape of rgb_data: (3, 480, 640)
    rgb = np.transpose(rgb_data, (1, 2, 0))
    bgr = rgb[:, :, ::-1]
    bgr_input = bgr / 255

    depth = np.array(h5f['depth'], dtype=np.float32)

    new_depth = fill_depth_colorization(bgr_input, depth, 0.1)
    new_depth = new_depth * 1000
    new_depth = new_depth.astype(int)
    max_depth = np.amax(new_depth)
    min_depth = np.amin(new_depth)
    colored_depth = np.array(colored_depthmap(new_depth, min_depth, max_depth), dtype=np.uint8)
    colored_depth = colored_depth[:, :, ::-1]

    save_file_path = os.path.join(save_path, file[:-2]) + 'png'
    print(save_file_path)
    cv2.imwrite(save_file_path, colored_depth)

    np.savetxt(os.path.join(save_path, file[:-2]) + 'csv', new_depth, delimiter=',')


def save_projected_img(h5_path, file, save_path):
    '''
     :param h5_path, file name, save_path:
     :return: void
     save bgr img with colorized LiDAR projected
     '''
    h5f = h5py.File(h5_path, "r")
    rgb_data = np.array(h5f['rgb'], dtype=np.uint8)
    # Originally, shape of rgb_data: (3, 480, 640)
    rgb = np.transpose(rgb_data, (1, 2, 0))
    bgr = rgb[:, :, ::-1]
    bgr = np.ascontiguousarray(bgr, dtype=np.uint8) #Don't know why i should put this.

    '''Used for finding min and max'''
    depth = np.array(h5f['depth'], dtype=np.float32)
    max_depth = np.amax(depth)
    min_depth = np.amin(depth)

    max_depth = 3
    min_depth = 0

    projected = np.array(h5f['scan_projected'], dtype=np.float32)
    # max_depth = np.amax(projected)
    # min_depth = np.amin(projected)
    colored_pc = np.array(colored_depthmap(projected, min_depth, max_depth), dtype=np.uint8)
    colored_pc = colored_pc[:, :, ::-1]

    for i in range(480):
        for j in range(640):
            reference_color = colored_pc[0, 0, :].tolist()
            get_color = colored_pc[i, j, :].tolist()
            if get_color != reference_color:
                bgr = cv2.circle(bgr, (j, i), 5, get_color, -1)

    save_file_path = os.path.join(save_path, file[:-2]) + 'png'
    cv2.imwrite(save_file_path, bgr)


def save_depth_intensity_img(h5_path, save_path):

    h5f = h5py.File(h5_path, "r")
    depth = np.array(h5f['depth'], dtype=np.float32)
    max_depth = np.amax(depth)
    depth = depth / max_depth * 255
    depth = depth.astype(np.int32)

    save_file_path = save_path[:-3] + '.png'
    cv2.imwrite(save_file_path, depth)


if __name__ == "__main__":

    ## variable to change! ##
    dir_name = 'for_pcl'

    #for hj laptop - external SSD
    h5_file_path = '/media/hj/Samsung_T5/icra2020/'+ dir_name + '/h5'
    projected_img_save_path = '/media/hj/Samsung_T5/icra2020/'+ dir_name + '/img+rp'
    concat_save_path = '/media/hj/Samsung_T5/icra2020/'+ dir_name +'/concat'

    new_depth2csv_save_path = '/media/hj/Samsung_T5/icra2020/' + dir_name + '/new_depth2csv'
    depth_intensity_save_path = '/media/hj/Samsung_T5/icra2020/'

    fList = os.listdir(h5_file_path)
    new_list = sorted(fList, key=lambda x: int(x[:-3]))
    print(new_list)

    count = 0

    for csv_file in new_list:

        print(csv_file)
        print(os.path.join(h5_file_path, csv_file))

        save_projected_img(os.path.join(h5_file_path, csv_file), csv_file, projected_img_save_path)
        save_concated_img(os.path.join(h5_file_path, csv_file), projected_img_save_path + '/' + csv_file[:-3] + '.png', csv_file, concat_save_path)

        if csv_file in ['0010.h5', '0066.h5', '0101.h5', '0114.h5', '0144.h5', '0192.h5', '0232.h5', '0257.h5', '0309.h5', '0355.h5', '0382.h5']:
            save_depth_filled_csv(os.path.join(h5_file_path, csv_file), csv_file, new_depth2csv_save_path)

        save_depth_intensity_img(os.path.join(h5_file_path, csv_file), os.path.join(depth_intensity_save_path, csv_file))

        count += 1

        # if count == 1:
        #     break
