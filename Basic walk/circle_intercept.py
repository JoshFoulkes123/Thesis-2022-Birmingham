import numpy as np
import math
from matplotlib import pyplot as plt

point = np.array([[20],[30]])
angle = 0*(math.pi/180)
radius = 100

print(angle)


if angle == 0:
    angle = 180




m= math.cos(angle)/math.sin(angle)
c= point[1] - (m*point[0])

print(m)

a = -m
b  = 1
x = point[0]
y = point[1]

point1 = np.array([(a*c+b*math.sqrt(radius**2*(a**2+b**2)-c**2))/(a**2+b**2),(b*c - a*math.sqrt(radius**2*(a**2+b**2)-c**2))/(a**2+b**2)])
point2 = np.array([(a*c-b*math.sqrt(radius**2*(a**2+b**2)-c**2))/(a**2+b**2),(b*c + a*math.sqrt(radius**2*(a**2+b**2)-c**2))/(a**2+b**2)])
 
print(point1)
print(point2)

draw_circle = plt.Circle((0,0),radius,fill=False,edgecolor = "blue")

plt.plot(0,0,marker="x", markersize=20, markerfacecolor="black")
plt.plot(point[0],point[1],marker="o", markersize=10, markerfacecolor="black",markeredgecolor = "black")

plt.plot(point1[0],point1[1],marker="o", markersize=10, markerfacecolor="red",markeredgecolor = "red")
plt.plot(point2[0],point2[1],marker="o", markersize=10, markerfacecolor="red",markeredgecolor = "red")
plt.plot((point1[0],point2[0]),(point1[1],point2[1]),'r-',lw=2)



plt.plot(point[0],point[1],marker="o", markersize=10, markerfacecolor="black",markeredgecolor = "black")
plt.plot((point[0],point[0]+math.sin(angle)*40),(point[1],point[1]+math.cos(angle)*40),'k-',lw=4)


plt.xlim(xmin=-150,xmax=150)
plt.ylim(ymin=-150,ymax=150)
plt.gcf().gca().add_artist(draw_circle)
plt.show()