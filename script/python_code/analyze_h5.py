import h5py
import os
import numpy as np
import cv2
import matplotlib.pyplot as plt

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
    #print("Shape of rgb: ", rgb_data.shape)
    rgb = np.transpose(rgb_data, (1, 2, 0))
    bgr = rgb[:, :, ::-1]

    depth = np.array(h5f['depth'], dtype=np.float32)
    #print("Shape of depth: ", depth.shape)
    max_depth = np.amax(depth)
    min_depth = np.amin(depth)
    colored_depth = np.array(colored_depthmap(depth, min_depth, max_depth), dtype=np.uint8)
    #print("Shape of colored_depth: ", colored_depth.shape)
    colored_depth = colored_depth[:, :, ::-1]

    projected = np.array(h5f['scan_projected'], dtype=np.float32)
    #print("Shape of projected: ", projected.shape)
    # max_depth = np.amax(projected)
    # min_depth = np.amin(projected)
    colored_pc = np.array(colored_depthmap(projected, min_depth, max_depth), dtype=np.uint8)
    #print("Shape of colored_pc: ", colored_pc.shape)
    colored_pc = colored_pc[:, :, ::-1]
    im_result = cv2.hconcat([bgr, colored_depth, colored_pc])
    cv2.imshow("visualized", im_result)
    cv2.waitKey(0)
    # cv2.destroyAllWindows()


def visualize_h5f2(h5_path):
    '''
     :param h5_path:
     :return: void
     NOT USED! needs modification.
     Basically, First attempt was to use ONLY this function to save final image.
     Problem : bgr type changed to CV_8SC3 (Don't know how to convert it to CV_8UC3 again.)
     '''
    h5f = h5py.File(h5_path, "r")
    rgb_data = np.array(h5f['rgb'], dtype=np.uint8)
    # Originally, shape of rgb_data: (3, 480, 640)
    #print("Shape of rgb: ", rgb_data.shape)
    rgb = np.transpose(rgb_data, (1, 2, 0))
    bgr = rgb[:, :, ::-1]
    # print(type((255,0,0)))
    # bgr_new = cv2.circle(bgr, (100, 100), 10, (255,0,0),-1)
    depth = np.array(h5f['depth'], dtype=np.float32)
    #print("Shape of depth: ", depth.shape)
    max_depth = np.amax(depth)
    min_depth = np.amin(depth)
    colored_depth = np.array(colored_depthmap(depth, min_depth, max_depth), dtype=np.uint8)
    #print("Shape of colored_depth: ", colored_depth.shape)
    colored_depth = colored_depth[:, :, ::-1]

    projected = np.array(h5f['scan_projected'], dtype=np.float32)
    #print("Shape of projected: ", projected.shape)
    # max_depth = np.amax(projected)
    # min_depth = np.amin(projected)
    colored_pc = np.array(colored_depthmap(projected, min_depth, max_depth), dtype=np.uint8)
    print("Shape of colored_pc: ", colored_pc.shape)
    colored_pc = colored_pc[:, :, ::-1]
    for i in range(480):
        for j in range(640):
            #get_color = tuple(map(int, colored_pc[i, j, :]))
            reference_color = colored_pc[0, 0, :].tolist()
            get_color = colored_pc[i, j, :].tolist()
            #print(get_color, type(get_color))
            if get_color != reference_color:
                bgr = cv2.circle(bgr, (j, i), 2, get_color, -1)

    bgr = np.asarray(bgr)
    im_result = cv2.hconcat([bgr, colored_depth, colored_pc])
    cv2.imshow("visualized", im_result)
    #cv2.imshow("bgr", bgr_new)
    cv2.waitKey(0)
    # cv2.destroyAllWindows()


def visualize_h5f_nyu(h5_path):
    '''
    :param h5_path:
    :return: void
    visualize bgr + colorized depth
    '''
    h5f = h5py.File(h5_path, "r")
    rgb_data = np.array(h5f['rgb'], dtype=np.uint8)
    # Originally, shape of rgb_data: (3, 480, 640)
    #print("Shape of rgb: ", rgb_data.shape)
    rgb = np.transpose(rgb_data, (1, 2, 0))
    bgr = rgb[:, :, ::-1]

    depth = np.array(h5f['depth'], dtype=np.float32)
    #print("Shape of depth: ", depth.shape)
    max_depth = np.amax(depth)
    min_depth = np.amin(depth)
    colored_depth = np.array(colored_depthmap(depth, min_depth, max_depth), dtype=np.uint8)
    #print("Shape of colored_depth: ", colored_depth.shape)
    colored_depth = colored_depth[:, :, ::-1]

    im_result = cv2.hconcat([bgr, colored_depth])
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



def visualize_depth_filled_h5f(h5_path):
    '''
     :param h5_path:
     :return: void
     visualize bgr + depth-filled colorized depth + colorized LiDAR
     '''
    h5f = h5py.File(h5_path, "r")
    rgb_data = np.array(h5f['rgb'], dtype=np.uint8)
    # Originally, shape of rgb_data: (3, 480, 640)
    print("Shape of rgb: ", rgb_data.shape)
    rgb = np.transpose(rgb_data, (1, 2, 0))
    bgr = rgb[:, :, ::-1]

    bgr_input = bgr / 255
    assert isinstance(bgr_input.shape, object)
    #print("Shape of bgr_input: ", bgr_input.shape)

    depth = np.array(h5f['depth'], dtype=np.float32)
    #print("Shape of depth: ", depth.shape)
    max_depth = np.amax(depth)
    min_depth = np.amin(depth)
    new_depth = fill_depth_colorization(bgr_input, depth, 1)
    colored_depth = np.array(colored_depthmap(new_depth, min_depth, max_depth), dtype=np.uint8)
    colored_depth = colored_depth[:, :, ::-1]

    projected = np.array(h5f['scan_projected'], dtype=np.float32)
    #print("Shape of projected: ", projected.shape)
    # max_depth = np.amax(projected)
    # min_depth = np.amin(projected)
    colored_pc = np.array(colored_depthmap(projected, min_depth, max_depth), dtype=np.uint8)
    #print("Shape of colored_pc: ", colored_pc.shape)
    colored_pc = colored_pc[:, :, ::-1]

    im_result = cv2.hconcat([bgr, colored_depth, colored_pc])
    cv2.imshow("visualized", im_result)
    cv2.waitKey(0)
    # cv2.destroyAllWindows()


def save_h5f2RGB(h5_path, file, save_path):
    '''
     :param h5_path, file, save_path:
     :return: void
     save concated bgr + colorized depth + colorized LiDAR as .png
     '''
    h5f = h5py.File(h5_path, "r")
    rgb_data = np.array(h5f['rgb'], dtype=np.uint8)
    # Originally, shape of rgb_data: (3, 480, 640)
    #print("Shape of rgb: ", rgb_data.shape)
    rgb = np.transpose(rgb_data, (1, 2, 0))
    bgr = rgb[:, :, ::-1]

    depth = np.array(h5f['depth'], dtype=np.float32)
    #print("Shape of depth: ", depth.shape)
    max_depth = np.amax(depth)
    min_depth = np.amin(depth)
    colored_depth = np.array(colored_depthmap(depth, min_depth, max_depth), dtype=np.uint8)
    #print("Shape of colored_depth: ", colored_depth.shape)
    colored_depth = colored_depth[:, :, ::-1]

    projected = np.array(h5f['scan_projected'], dtype=np.float32)
    #print("Shape of projected: ", projected.shape)
    # max_depth = np.amax(projected)
    # min_depth = np.amin(projected)
    colored_pc = np.array(colored_depthmap(projected, min_depth, max_depth), dtype=np.uint8)
    #print("Shape of colored_pc: ", colored_pc.shape)
    colored_pc = colored_pc[:, :, ::-1]
    im_result = cv2.hconcat([bgr, colored_depth, colored_pc])
    save_file_path = os.path.join(save_path, file[:-2]) + 'png'
    print(save_file_path)
    cv2.imwrite(save_file_path, im_result)

def save_new_h5(h5_path, file, save_path):
    '''
     :param h5_path, file, save_path:
     :return: void
     save depth_filled colorized depth as .png
     '''
    h5f = h5py.File(h5_path, "r")
    rgb_data = np.array(h5f['rgb'], dtype=np.uint8)
    # Originally, shape of rgb_data: (3, 480, 640)
    #print("Shape of rgb: ", rgb_data.shape)
    rgb = np.transpose(rgb_data, (1, 2, 0))
    bgr = rgb[:, :, ::-1]
    bgr_input = bgr / 255

    depth = np.array(h5f['depth'], dtype=np.float32)
    #print("Shape of depth: ", depth.shape)

    new_depth = fill_depth_colorization(bgr_input, depth, 1)
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


def save_concat_img(h5_path, img_path, file, save_path):
    '''
     :param h5_path, img_path, file, save_path:
     :return: void
     concat LiDAR-projected bgr img + colorized depth img  and save it as .png
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
    save_file_path = os.path.join(save_path, file[:-3]) + '.png'
    print(save_file_path)
    cv2.imwrite(save_file_path, im_result)


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

    max_depth = 5
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
                bgr = cv2.circle(bgr, (j, i), 2, get_color, -1)

    #bgr = np.asarray(bgr)

    save_file_path = os.path.join(save_path, file[:-2]) + 'png'
    #print(save_file_path)
    cv2.imwrite(save_file_path, bgr)


def show_raw_data_shape(h5_path):
    h5f = h5py.File(h5_path, "r")
    raw = np.array(h5f['scan_raw'], dtype=np.float32)
    print(h5_path, raw.shape)



def show_keys_of_h5(h5_path):
    h5f = h5py.File(h5_path, "r")
    print(list(h5f.keys()))


#
def get_nonzero(depth):
    return np.count_nonzero(depth)


def calculate_nonzero_mean_std(file_list):
    pass


if __name__ == "__main__":

    ## variable to change! ##
    dir_name = 'cafeteria_n0'

    # #for shapelim laptop
    # h5_file_path = '/media/shapelim/SAMSUNG/icra2020/'+ dir_name + '/h5'
    # projected_img_save_path = '/media/shapelim/SAMSUNG/icra2020/'+ dir_name + '/img+rp'
    # concat_save_path = '/media/shapelim/SAMSUNG/icra2020/'+ dir_name +'/concat'

    # #for hj laptop - external HDD
    # h5_file_path = '/media/hj/SAMSUNG/icra2020/'+ dir_name + '/h5'
    # projected_img_save_path = '/media/hj/SAMSUNG/icra2020/'+ dir_name + '/img+rp'
    # concat_save_path = '/media/hj/SAMSUNG/icra2020/'+ dir_name +'/concat'

    #for hj laptop - external SSD
    h5_file_path = '/media/hj/Samsung_T5/icra2020/'+ dir_name + '/h5'
    projected_img_save_path = '/media/hj/Samsung_T5/icra2020/'+ dir_name + '/img+rp'
    concat_save_path = '/media/hj/Samsung_T5/icra2020/'+ dir_name +'/concat'

    #for hj laptop - ssd
    # h5_file_path = '/home/hj/icra2020/'+ dir_name + '/h5'
    # projected_img_save_path = '/home/hj/icra2020/'+ dir_name + '/img+rp'
    # concat_save_path = '/home/hj/icra2020/'+ dir_name +'/concat'

    save_path = '/media/hj/Samsung_T5/icra2020/' + dir_name + '/new_depth2csv'


    val_h5_file_path = '/media/hj/Samsung_T5/url_proto/val'

    fList = os.listdir(h5_file_path)
    new_list = sorted(fList, key=lambda x: int(x[:-3]))
    print(new_list)

    count = 0

    for csv_file in new_list:

        # print(csv_file)
        # print(os.path.join(h5_file_path, csv_file))

        # save_projected_img(os.path.join(h5_file_path, csv_file), csv_file, projected_img_save_path)
        #visualize_depth_and_projected_img(projected_img_save_path + '/' + csv_file[:-3] + '.png', os.path.join(h5_file_path, csv_file))
        # save_concat_img(os.path.join(h5_file_path, csv_file), projected_img_save_path + '/' + csv_file[:-3] + '.png', csv_file, concat_save_path)


        # show_keys_of_h5(os.path.join(h5_file_path, csv_file))
        # save_h5f2RGB(os.path.join(h5_file_path, csv_file), csv_file, save_path)
        # if csv_file in ['0010.h5', '0066.h5', '0101.h5', '0114.h5', '0144.h5', '0192.h5', '0232.h5', '0257.h5', '0309.h5', '0355.h5', '0382.h5']:
        #     save_new_h5(os.path.join(h5_file_path, csv_file), csv_file, save_path)

        #visualize_h5f2(os.path.join(h5_file_path, csv_file))
        #visualize_h5f_nyu(os.path.join(h5_file_path, csv_file))

        # show_raw_data_shape(os.path.join(val_h5_file_path, csv_file))

        count += 1

        # if count == 2:
        #     break
