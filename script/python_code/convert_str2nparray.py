import csv
import numpy as np

def convert_img2csv_str2nparray(csv_abs_path: object):
    '''
    :param csv_abs_path: abs path of img2csv file
    :return: csv file converted to numpy
    '''
    f=open(csv_abs_path, 'r', encoding='utf-8')
    rdr = csv.reader(f)
    count = 0
    list = []
    for line in rdr:
        line = [float(i) for i in line[:-1]]
        if len(line) == 0:
            print("!!")
            break
        list.append(line)
        # break
        count += 1
    f.close()
    np_array =np.array(list)
    return np_array

def convert_depth2csv_str2nparray(csv_abs_path: object):
    '''
    :param csv_abs_path: abs path of img2csv file
    :return: csv file converted to numpy
    '''
    f=open(csv_abs_path, 'r', encoding='utf-8')
    rdr = csv.reader(f)
    count = 0
    list = []
    for line in rdr:
        line = [float(i)/1000 for i in line[:]]
        if len(line) == 0:
            print("!!")
            break
        list.append(line)
        # break
        count += 1
    f.close()
    np_array =np.array(list)
    return np_array

def convert_csv_str2nparray(csv_abs_path):
    '''

    :param csv_abs_path: abs path of csv file
    :return: csv file converted to numpy
    '''
    f=open(csv_abs_path, 'r', encoding='utf-8')
    rdr = csv.reader(f)
    count = 0
    list = []
    for line in rdr:
        line = [float(i) for i in line]

        list.append(line)
        # break
        count += 1
    f.close()
    np_array =np.array(list)
    return np_array

if __name__ == "__main__":
    pass