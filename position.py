import json
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import ctypes
from ctypes import *
import numpy as np
import numpy.ctypeslib as npct

fig,ax=plt.subplots()

load_f = open("./position.json",'r')
load_dict = json.load(load_f)
# print(load_dict)

segs = load_dict["position"]
# print(segs)
n = len(segs)

INPUT = c_float * (4*n) 
data = INPUT()
id = 0
for i in range(n):
    seg_obj = segs[i]
    if len(seg_obj) == 0:
        continue
    
    seg_list = seg_obj.values()[0]
    # if (i==0):
        # print (seg_list)
    lx = []
    ly = []
    for j in range (2):
        p_dict = seg_list[j]
        x = float(p_dict["x"])
        y = float(p_dict["y"])
        lx.append(x)
        ly.append(y)
        data[id] = x
        id+=1
        data[id] = y
        id+=1
    plt.plot(lx, ly)

#Spacing between each line
intervals = 0.01

loc = plticker.MultipleLocator(base=intervals)
ax.xaxis.set_major_locator(loc)
ax.yaxis.set_major_locator(loc)
plt.grid(1, linewidth = 1, c = 'r' )

# plt.show()

func = CDLL('./libProject.so')
func.tri.restype = c_int
T_int_ptr = c_void_p(0)
cn = c_int32(n)

while True:
    result=func.tri(data, cn, byref(T_int_ptr))
    print(T_int_ptr)
    print(result)

    g = (ctypes.c_int32*result).from_address(T_int_ptr.value)
    for i in range(result):
        print(g[i])

    func.freeIndex(T_int_ptr)

