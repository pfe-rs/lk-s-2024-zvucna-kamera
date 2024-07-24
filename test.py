import numpy as np


l1 =[1,1,0,1,-1,0,0.5,1,2,0,0,-0.7,1]
l2 =[1,1,0,1,-1,0,0.5,1,2,0,0,-0.7,1]
l2 = np.roll(l2, 7)
_max = -100000000000000000
for i in range(len(l1)):
    temp1 = np.dot(l1, np.roll(l2, i))#np.concatenate((np.roll(signal[1], i)[i:], np.zeros(i,dtype = int))))
    if i == 0:
        temp2 = -1000000
    else:
        temp2 = np.dot(l1, np.roll(l2, -i))#np.concatenate((np.zeros(i,dtype = int), np.roll(signal[1], -i)[:-i])))
    if temp1 > _max:
        _max = temp1
        print(i, temp1)
        tdoa = i
    if temp2 > _max:
        _max = temp2
        print(-i, temp2)
        tdoa = -i

print(l1)
print(l2)

print(tdoa)
