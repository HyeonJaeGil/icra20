import scipy.stats
import os
import csv
import numpy as np
import matplotlib.pyplot as plt
from PlotWeight import convert_csv2list

'''
Shows mean and std of the number of lidar points in each image
'''



rel_path = '/home/hj/ICRA2020/190403/rp_img2csv/'
rel_path2 = '/home/hj/ICRA2020/190403/rp_raw/'
rel_path3 = '/home/hj/ICRA2020/190403/rp_intensity/'
fList = os.listdir(rel_path)
NewList = sorted(fList, key=lambda x: int(x[7:-4]))
List1=[]
List2=[]
List3=[]

count = 0

for file in NewList:
    path1 = rel_path + file
    path2 = rel_path2 + file
    path3 = rel_path3 + file
    ElementList1 = convert_csv2list(path1)
    ElementList2 = convert_csv2list(path2)
    ElementList3 = convert_csv2list(path3)


    NonZero = np.count_nonzero(ElementList1)
    List1.append(NonZero)
    Length2 = len(ElementList2)
    List2.append(Length2)
    Length3 = len(ElementList2)
    List3.append(Length3)

    count += 1
    #if count == 350:
        #print(List1)
        #break

count = 0

Array1 = np.array(List1)
print(np.mean(Array1))
print(np.std(Array1))

Array2 = np.array(List2)
print(np.mean(Array2))
print(np.std(Array2))

Array3 = np.array(List3)
print(np.mean(Array3))
print(np.std(Array3))

SortedArray1 = sorted(Array1)
SortedArray2 = sorted(Array2)
SortedArray3 = sorted(Array3)
# print(SortedArray1)
# print(SortedArray2)
# print(SortedArray3)

# plt.hist(SortedArray1, bins=100)
# plt.title("img2csv")
# fig2 = plt.figure(2)
# plt.hist(SortedArray2, bins=100)
# plt.title("raw")
# plt.show()

print(List2)

Max =max(List2)
print(List2.index(Max))

Max =max(List3)
print(List3.index(Max))