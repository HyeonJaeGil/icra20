import scipy.stats
import os
import csv
import numpy as np
import matplotlib.pyplot as plt
import math


def convert_csv2list(csv_abs_path):
    '''
    param: csv_abs_path (abs path of csv file)
    return: csv file converted to list
    '''

    f = open(csv_abs_path, 'r', encoding='utf-8')
    rdr = csv.reader(f)
    list = []
    for line in rdr:
        line = [float(i) for i in line[:-1]]
        list.append(line)
        # break
    f.close()
    return list


def get_angle_list(rel_path):
    f_list = os.listdir(rel_path)
    new_list = sorted(f_list, key=lambda x: int(x[7:-4]))
    angle_list = []
    count = 0
    for file in new_list:
        path = rel_path + file
        element_list = convert_csv2list(path)
        length = len(element_list)
        for j in range(0, length):
            if not element_list[j][0] in angle_list:
                print("New angle", element_list[j][0], "in", file)
                if element_list[j][0] > 0:
                    angle_list.insert(0, element_list[j][0])
                else:
                    angle_list.append(element_list[j][0])

        count += 1
    return angle_list


def get_weight_array(rel_path, angle_list):
    arr_length = len(angle_list)
    f_list = os.listdir(rel_path)
    new_list = sorted(f_list, key=lambda x: int(x[7:-4]))
    count = 0
    weight_array = np.zeros(arr_length)
    for file in new_list:  # for every files
        path = rel_path + file
        element_list = convert_csv2list(path)
        length = len(element_list)
        for i in range(length):
            if element_list[i][0] in angle_list:
                weight_array[i] += 1
        count += 1
    return weight_array


if __name__ == '__main__':

    rel_path = '/home/hj/ICRA2020/190403/rp_raw/'

    AngleList = get_angle_list(rel_path)

    AngleArray = np.array(AngleList)

    WeightArray = get_weight_array(rel_path, AngleList)

    XArray = AngleArray * 180 / math.pi

    # ArrLength = len(AngleList)
    # for i in range(ArrLength):
    #     if XArray[i] < 0:
    #         XArray[i] += 360

    print(np.amax(XArray))

    plt.bar(XArray, WeightArray, label='Set 1', color='blue', width=0.1)
    #plt.plot(XArray,WeightArray)
    plt.title("Weight of each angle")
    plt.show()
