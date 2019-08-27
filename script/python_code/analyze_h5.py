from save_img_from_h5 import *
from visualize_img_from_h5 import *

def show_raw_data_shape(h5_path):
    h5f = h5py.File(h5_path, "r")
    raw = np.array(h5f['scan_raw'], dtype=np.float32)
    print(h5_path, raw.shape)


def show_keys_of_h5(h5_path):
    h5f = h5py.File(h5_path, "r")
    print(list(h5f.keys()))


def get_nonzero(depth):
    return np.count_nonzero(depth)


if __name__ == "__main__":

    ## variable to change! ##
    dir_name = 'for_pcl'

    #for hj laptop - external SSD
    h5_file_path = '/media/hj/Samsung_T5/icra2020/'+ dir_name + '/h5'
    projected_img_save_path = '/media/hj/Samsung_T5/icra2020/'+ dir_name + '/img+rp'
    concat_save_path = '/media/hj/Samsung_T5/icra2020/'+ dir_name +'/concat'

    new_depth2csv_save_path = '/media/hj/Samsung_T5/icra2020/' + dir_name + '/new_depth2csv'


    fList = os.listdir(h5_file_path)
    new_list = sorted(fList, key=lambda x: int(x[:-3]))
    print(new_list)

    count = 0

    for csv_file in new_list:

        print(csv_file)
        print(os.path.join(h5_file_path, csv_file))

        save_projected_img(os.path.join(h5_file_path, csv_file), csv_file, projected_img_save_path)
        visualize_depth_and_projected_img(projected_img_save_path + '/' + csv_file[:-3] + '.png', os.path.join(h5_file_path, csv_file))
        save_concated_img(os.path.join(h5_file_path, csv_file), projected_img_save_path + '/' + csv_file[:-3] + '.png', csv_file, concat_save_path)

        show_keys_of_h5(os.path.join(h5_file_path, csv_file))

        if csv_file in ['0010.h5', '0066.h5', '0101.h5', '0114.h5', '0144.h5', '0192.h5', '0232.h5', '0257.h5', '0309.h5', '0355.h5', '0382.h5']:
            save_depth_filled_csv(os.path.join(h5_file_path, csv_file), csv_file, new_depth2csv_save_path)

        visualize_h5f(os.path.join(h5_file_path, csv_file))

        show_raw_data_shape(os.path.join(h5_file_path, csv_file))

        save_depth_intensity_img(os.path.join(h5_file_path, csv_file), os.path.join(save_path, csv_file))

        count += 1

        # if count == 1:
        #     break

