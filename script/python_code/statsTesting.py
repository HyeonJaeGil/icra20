import scipy.stats
import os
import csv
import numpy as np


'''
1. make the file names to np.array (or list)
2. for each img2csv file, count the nonzero elements
3. append the result to new list
4. for each raw file, count the number of rows
5. append the result to new list
'''

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


rel_path1 = '/home/hj/ICRA2020/190403/rp_img2csv/'
fList1 = os.listdir(rel_path1)
NewList1 = sorted(fList1, key=lambda x: int(x[7:-4]))
List1=[]

rel_path2 = '/home/hj/ICRA2020/190403/rp_intensity/'
fList2 = os.listdir(rel_path2)
NewList2 = sorted(fList2, key=lambda x: int(x[7:-4]))
List2=[]

count = 0

for file in NewList1:
    path1 = rel_path1 + file
    ElementList1 = convert_csv2list(path1)
    NonZero = np.count_nonzero(ElementList1)
    List1.append(NonZero)
    count += 1
    #if count == 1:
        #print(List1)
        #break

count = 0

for file in NewList2:
    path2 = rel_path2 + file
    ElementList2 = convert_csv2list(path2)
    Length = len(ElementList2)
    List2.append(Length)
    count += 1
    #if count == 1:
        #print(List1)
        #break

Array1 = np.array(List1)
print(np.mean(Array1))
print(np.std(Array1))

Array2 = np.array(List2)
print(np.mean(Array2))
print(np.std(Array2))
