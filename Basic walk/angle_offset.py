import numpy as np
import math
from matplotlib import pyplot as plt


radius = 4
change = -90 * (math.pi/180)

point = np.array([2,2])
center = np.array([1,1])

print(math.atan2(point[0],point[1])*(180/math.pi))

plt.plot(point[0],point[1],marker="x", markersize=20, markerfacecolor="black")

plt.show()
plt.plot(point[0],point[1],marker="x", markersize=20, markerfacecolor="black")


val = np.array([radius*math.sin(change+math.atan2(point[0],point[1])),radius*math.cos(change+math.atan2(point[0],point[1]))])

plt.plot(val[0],val[1],marker=".", markersize=3, markerfacecolor="black")
plt.plot((point[0],0),(point[1],0),'k-',lw=3)
plt.plot((val[0],0),(val[1],0),'k-',lw=3)

print(((math.atan2(point[0],point[1]))-(math.atan2(val[0],val[1])))*(180/math.pi))
    
plt.show()