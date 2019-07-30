import h5py
import os
import numpy as np
import cv2
import sys
from convert_str2nparray import convert_img2csv_str2nparray, convert_depth2csv_str2nparray


def generate_h5(img_file_path, rp_img2csv_path, output_dir):
    img = cv2.imread(img_file_path)
    depth_np = convert_img2csv_str2nparray(rp_img2csv_path)
    assert isinstance(depth_np.shape, object)
    print(depth_np.shape)

    hf = h5py.File(output_dir, 'w')
    # Make it reverse is for mathing the format of sparse-to-depth prediction code
    hf.create_dataset("rgb", data=np.transpose(img, (2, 0, 1))[::-1, :, :])
    hf.create_dataset("depth", data=depth_np)
    # raw, csv2img
    hf.close()


def generate_our_h5(rel_path, file_name):

    img_path = 'img'
    rp_img2csv_path = 'rp_img2csv'
    rp_raw_path = 'rp_raw'
    rp_intensity_path = 'rp_intensity'
    depth_path = 'depth2csv'
    output_path = 'h5'


    img_file_path = os.path.join(rel_path, img_path, file_name) + '.png'
    rp_img2csv_file_path = os.path.join(rel_path, rp_img2csv_path, file_name) + '.csv'
    rp_raw_file_path = os.path.join(rel_path, rp_raw_path, file_name) + '.csv'
    rp_intensity_file_path = os.path.join(rel_path, rp_intensity_path, file_name) + '.csv'
    depth_file_path = os.path.join(rel_path, depth_path, file_name) + '.csv'
    output_file_path = os.path.join(rel_path, output_path, file_name) + '.h5'

    img = cv2.imread(img_file_path)

    scan_projected_np = convert_img2csv_str2nparray(rp_img2csv_file_path)
    assert isinstance(scan_projected_np.shape, object)
    print(scan_projected_np.shape)

    scan_raw_np = convert_img2csv_str2nparray(rp_raw_file_path)
    assert isinstance(scan_raw_np.shape, object)
    print(scan_raw_np.shape)

    scan_intensity_np = convert_img2csv_str2nparray(rp_intensity_file_path)
    assert isinstance(scan_intensity_np.shape, object)
    print(scan_intensity_np.shape)

    depth_np = convert_depth2csv_str2nparray(depth_file_path)
    assert isinstance(depth_np.shape, object)
    print(depth_np.shape)


    hf = h5py.File(output_file_path, 'w')
    # Make it reverse is for mathing the format of sparse-to-depth prediction code
    hf.create_dataset("rgb", data=np.transpose(img, (2, 0, 1))[::-1, :, :])
    hf.create_dataset("scan_projected", data=scan_projected_np)
    hf.create_dataset("scan_raw", data=scan_raw_np)
    hf.create_dataset("scan_intensity", data=scan_intensity_np)
    hf.create_dataset("depth", data=depth_np)

    # scan_raw
    # scan_projected
    # scan_intensity
    # raw, csv2img
    hf.close()


if __name__ == "__main__":
    target_path = '/home/hj/ICRA2020/190403'
    #img_path ='img'
    #rp_img2csv_path = 'rp_img2csv'

    #img_folder_path = os.path.join(target_path, img_path)
    #rp_img2csv_folder_path = os.path.join(target_path, rp_img2csv_path)

    #print(img_folder_path)
    #print(rp_img2csv_folder_path)

    f_list = os.listdir(target_path+'/img')
    new_list = sorted(f_list, key=lambda x: int(x[7:-4]))

    count = 0

    for file_name in new_list:
        file_name = file_name[:-4]
        print(file_name)

        generate_our_h5(target_path, file_name)

        count += 1
        print("Saving for %d trial done.", count)
        #if count == 10:
            #break

