import os
import h5py
import numpy as np
import cv2
import open3d as o3d
from generate_pointcloud import generate_pointcloud



def get_bgr_and_depth(h5f_path):
    h5f = h5py.File(h5f_path, 'r')

    depth = np.array(h5f['depth'], dtype=np.float32)

    rgb = np.array(h5f['rgb'], dtype=np.uint8)
    rgb = np.transpose(rgb, (1, 2, 0))
    bgr = rgb[:, :, ::-1]
    h5f.close()
    return bgr, depth


def save_img_from_h5(img, depth_img, img_save_abs_path, depth_img_save_abs_path):

    max_depth = np.amax(depth_img)
    depth = depth_img / max_depth * 255
    depth = depth.astype(np.int32)

    cv2.imwrite(img_save_abs_path, img)
    cv2.imwrite(depth_img_save_abs_path, depth)


def visualize_ply(ply_path):
    pcd = o3d.read_point_cloud(ply_path) # Read the point cloud
    o3d.visualization.draw_geometries([pcd]) # Visualize the point cloud


if __name__ == "__main__":

    rel_path = '/media/hj/Samsung_T5/pcl/unknown_e9_4'
    h5_path = rel_path + '/h5'
    fList = os.listdir(h5_path)
    new_list = sorted(fList, key=lambda x: int(x[:-3]))

    img_save_rel_path = rel_path + '/img'
    depth_img_save_rel_path = rel_path + '/depth_img'
    ply_save_rel_path = rel_path + '/ply'

    for file in new_list:
        abs_file_path = os.path.join(h5_path, file)
        h5f = h5py.File(abs_file_path, 'r')
        bgr, depth = get_bgr_and_depth(abs_file_path)

        img_save_path = os.path.join(img_save_rel_path, file[:-3]) + '.png'
        depth_img_save_path = os.path.join(depth_img_save_rel_path, file[:-3]) + '.png'
        save_img_from_h5(bgr, depth, img_save_path, depth_img_save_path)

        ply_save_path = os.path.join(ply_save_rel_path, file[:-3]) + '.ply'
        generate_pointcloud(img_save_path, depth_img_save_path, ply_save_path)

        visualize_ply(ply_save_path)