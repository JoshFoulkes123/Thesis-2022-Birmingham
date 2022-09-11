import numpy as np
import math
from matplotlib import pyplot as plt
from matplotlib import patches as pth

print("hello world")

_radius = 200

i = 0

_speed = 0.08 #degrees per iteration

_time = 300


radius_increase = 1

leg_Positions  = np.array([[115,-70,0],[55,-140,0],[-55,-140,0],[-115,-70,0],[115,70,0],[55,140,0],[-55,140,0],[-115,70,0]])


edge_dict = np.array([[-45,0],[-90,-45],[-135,-90],[-180,-135],[45,0],[90,45],[135,90],[180,135]])

# distance = speed*time

# print("Distance: "+str(distance))

# radiuses = np.ones(8)*radius

def start_stop_turn(radius,Speed,time,leg_index):
    speed = abs(Speed)
    i = leg_index
    distance = speed*time
    radiuses = np.ones(8)*radius
    
    angle = (edge_dict[i,0]-edge_dict[i,1])/2+edge_dict[i,1]
    point_mid = np.array([math.sin(-(angle*(math.pi/180)-(math.pi/2)))*100,math.cos(-(angle*(math.pi/180)-(math.pi/2)))*100])


    if angle == 0:
        angle = 180
    angle = (-(angle-90))*(math.pi/180)

    point = leg_Positions[i]
    print(point)

    m= math.cos(angle)/math.sin(angle)


    c= point[1] - (m*point[0])

    a = -m
    b  = 1
    x = point[1]
    y = point[0]

    point1 = np.array([(a*c+b*math.sqrt(radius**2*(a**2+b**2)-c**2))/(a**2+b**2),(b*c - a*math.sqrt(radius**2*(a**2+b**2)-c**2))/(a**2+b**2)])
    point2 = np.array([(a*c-b*math.sqrt(radius**2*(a**2+b**2)-c**2))/(a**2+b**2),(b*c + a*math.sqrt(radius**2*(a**2+b**2)-c**2))/(a**2+b**2)])

    min_ = min(edge_dict[i,0],edge_dict[i,1])
    max_ = max(edge_dict[i,0],edge_dict[i,1])


    if math.sqrt((point1[0] - point[0])**2+(point1[1] - point[1])**2) < math.sqrt((point2[0] - point[0])**2+(point2[1] - point[1])**2):
        edge_point = point1
    else:
        edge_point = point2
        
    change = (distance/2)*(math.pi/180)
    end_point1 = np.array([radiuses[i]*math.sin(change+math.atan2(edge_point[0],edge_point[1])),radiuses[i]*math.cos(change+math.atan2(edge_point[0],edge_point[1]))])
    end_point2 = np.array([radiuses[i]*math.sin(-change+math.atan2(edge_point[0],edge_point[1])),radiuses[i]*math.cos(-change+math.atan2(edge_point[0],edge_point[1]))])
    # print(range(*sorted((min_,max_))))
    idx = 0
    # print(-math.atan2(end_point1[0] - point[0],end_point1[1]- point[1])*(180/math.pi)+90,int(-math.atan2(end_point1[0] - point[0],end_point1[1]- point[1])*(180/math.pi)+90))
    # print(math.atan2(end_point1[1]- point[1],end_point1[0] - point[0])*(180/math.pi),int(-math.atan2(end_point1[0] - point[0],end_point1[1]- point[1])*(180/math.pi)+90))
    # print(-math.atan2(end_point2[0] - point[0],end_point2[1]- point[1])*(180/math.pi)+90,int(-math.atan2(end_point2[0] - point[0],end_point2[1]- point[1])*(180/math.pi)+90))    
    #while (int(converter.convert_angle(angles_start[0]*(math.pi/180),leg_index)*(180/math.pi)) not in range(*sorted((int(min_),int(max_))))) or (int(converter.convert_angle(angles_end[0]*(math.pi/180),leg_index)*(180/math.pi)) not in range(*sorted((int(min_),int(max_))))) or angles_end[0] == 404 or angles_start[0] == 404:
    while int(math.atan2(end_point1[1]- point[1],end_point1[0] - point[0])*(180/math.pi)) not in range(*sorted((min_,max_))) or int(math.atan2(end_point2[1]- point[1],end_point2[0] - point[0])*(180/math.pi)) not in range(*sorted((min_,max_))):
        radiuses[i] = radiuses[i]+radius_increase
        end_point1 = np.array([radiuses[i]*math.sin(change+math.atan2(edge_point[0],edge_point[1])),radiuses[i]*math.cos(change+math.atan2(edge_point[0],edge_point[1]))])
        end_point2 = np.array([radiuses[i]*math.sin(-change+math.atan2(edge_point[0],edge_point[1])),radiuses[i]*math.cos(-change+math.atan2(edge_point[0],edge_point[1]))]) 
        idx = idx+1
        if idx > 10000:
            break
    # print(idx)
    
    # print(-math.atan2(end_point1[0] - point[0],end_point1[1]- point[1])*(180/math.pi)+90,int(-math.atan2(end_point1[0] - point[0],end_point1[1]- point[1])*(180/math.pi)+90))
    # print(-math.atan2(end_point2[0] - point[0],end_point2[1]- point[1])*(180/math.pi)+90,int(-math.atan2(end_point2[0] - point[0],end_point2[1]- point[1])*(180/math.pi)+90))
    
    # print(math.atan2(end_point1[0],end_point1[1])*(180/math.pi))
    # print(math.atan2(point[0],point[1])*(180/math.pi))
    # print(math.atan2(end_point2[0],end_point2[1])*(180/math.pi))
    # print(((math.atan2(point[0],point[1]))-(math.atan2(end_point2[0],end_point2[1])))*(180/math.pi))
    # print(((math.atan2(point[0],point[1]))-(math.atan2(end_point1[0],end_point1[1])))*(180/math.pi))
    # print(((math.atan2(end_point1[0],end_point1[1]))-(math.atan2(end_point2[0],end_point2[1])))*(180/math.pi))

    if Speed > 0:
        start_point = end_point2
        end_point = end_point1
    else:
        start_point = end_point1
        end_point = end_point2    
    
    points = start_point
    val = np.array([radiuses[i]*math.sin(Speed*(math.pi/180)+math.atan2(points[0],points[1])),radiuses[i]*math.cos(Speed*(math.pi/180)+math.atan2(points[0],points[1]))])
    points = np.vstack((points,val))
    for q in range(_time-1):
        temp = points[q+1,:]
        val = np.array([radiuses[i]*math.sin(Speed*(math.pi/180)+math.atan2(temp[0],temp[1])),radiuses[i]*math.cos(Speed*(math.pi/180)+math.atan2(temp[0],temp[1]))])
        points = np.vstack((points,val))
        
    #===============Graph section ==============================

    draw_circle = plt.Circle((0,0),radius,fill=False,edgecolor = "black")
    plt.gcf().gca().add_artist(draw_circle)

    # plt.xlim(xmin=-max(radius+100,250),xmax=max(radius+100,250))
    # plt.ylim(ymin=-max(radius+100,250),ymax=max(radius+100,250))
    draw_arc = pth.Arc((0,0),radiuses[i]*2,radiuses[i]*2,min(-(math.atan2(end_point2[0],end_point2[1])*(180/math.pi)-90),-(math.atan2(end_point1[0],end_point1[1])*(180/math.pi)-90)),0,distance,fill=False,edgecolor = "red")
    plt.gcf().gca().add_artist(draw_arc)
    plt.plot(0,0,marker="x", markersize=20, markerfacecolor="red", markeredgecolor = "red")
    m= 0
    for n in edge_dict:
        point_start = np.array([math.sin(-(n[0]*(math.pi/180)-(math.pi/2)))*100,math.cos(-(n[0]*(math.pi/180)-(math.pi/2)))*100])
        point_end = np.array([math.sin(-(n[1]*(math.pi/180)-(math.pi/2)))*100,math.cos(-(n[1]*(math.pi/180)-(math.pi/2)))*100])
        plt.plot((point_start[0]+leg_Positions[m,0],leg_Positions[m,0]),(point_start[1]+leg_Positions[m,1],leg_Positions[m,1]),'k-',lw=3)
        plt.plot((point_end[0]+leg_Positions[m,0],leg_Positions[m,0]),(point_end[1]+leg_Positions[m,1],leg_Positions[m,1]),'k-',lw=3)
        m = m+1
        
        
    m= 0
     
    for n in leg_Positions:
        if m == i:
            plt.plot((point_mid[0]+leg_Positions[m,0],leg_Positions[m,0]),(point_mid[1]+leg_Positions[m,1],leg_Positions[m,1]),'b-',lw=2)
            plt.plot(n[0],n[1],marker=".", markersize=20, markerfacecolor="blue", markeredgecolor = "blue")
        else:
            plt.plot(n[0],n[1],marker=".", markersize=20, markerfacecolor="black", markeredgecolor = "black")
        m = m+1
        
    # plt.plot(point1[0],point1[1],marker="o", markersize=3, markerfacecolor="red",markeredgecolor = "red")
    # plt.plot(point2[0],point2[1],marker="o", markersize=3, markerfacecolor="red",markeredgecolor = "red")
    # plt.plot((point1[0],point2[0]),(point1[1],point2[1]),'r-',lw=2    )
    plt.plot(edge_point[0],edge_point[1],marker="o", markersize=8, markerfacecolor="red",markeredgecolor = "red")
    

        
    plt.plot(end_point[0],end_point[1],marker=".", markersize=8, markerfacecolor="cyan",markeredgecolor = "cyan")
    plt.plot(start_point[0],start_point[1],marker="*", markersize=8, markerfacecolor="cyan",markeredgecolor = "cyan")    
    for m in points:
       plt.plot(m[0],m[1],marker=".", markersize=8, markerfacecolor="magenta",markeredgecolor = "magenta") 
    
    plt.show()       
    
    
    return start_point,end_point,points



for i in range(1):

    print("==========================="+str(i+1)+"=========================")
  
    out = start_stop_turn(_radius,_speed,_time,i)
    print("start",out[0])
    print(out[2])
    print("end",out[1])
    

          
    
    
    
    # angle = (edge_dict[i,0]-edge_dict[i,1])/2+edge_dict[i,1]
    # point_mid = np.array([math.sin(-(angle*(math.pi/180)-(math.pi/2)))*100,math.cos(-(angle*(math.pi/180)-(math.pi/2)))*100])


    # if angle == 0:
        # angle = 180
    # angle = (-(angle-90))*(math.pi/180)

    # point = leg_Positions[i]

    # m= math.cos(angle)/math.sin(angle)


    # c= point[1] - (m*point[0])

    # a = -m
    # b  = 1
    # x = point[1]
    # y = point[0]

    # point1 = np.array([(a*c+b*math.sqrt(radius**2*(a**2+b**2)-c**2))/(a**2+b**2),(b*c - a*math.sqrt(radius**2*(a**2+b**2)-c**2))/(a**2+b**2)])
    # point2 = np.array([(a*c-b*math.sqrt(radius**2*(a**2+b**2)-c**2))/(a**2+b**2),(b*c + a*math.sqrt(radius**2*(a**2+b**2)-c**2))/(a**2+b**2)])

    # min_ = min(edge_dict[i,0],edge_dict[i,1])
    # max_ = max(edge_dict[i,0],edge_dict[i,1])


    # if math.sqrt((point1[0] - point[0])**2+(point1[1] - point[1])**2) < math.sqrt((point2[0] - point[0])**2+(point2[1] - point[1])**2):
        # edge_point = point1
    # else:
        # edge_point = point2
        
    # change = (distance/2)*(math.pi/180)
    # end_point1 = np.array([radiuses[i]*math.sin(change+math.atan2(edge_point[0],edge_point[1])),radiuses[i]*math.cos(change+math.atan2(edge_point[0],edge_point[1]))])
    # end_point2 = np.array([radiuses[i]*math.sin(-change+math.atan2(edge_point[0],edge_point[1])),radiuses[i]*math.cos(-change+math.atan2(edge_point[0],edge_point[1]))])
    # # print(range(*sorted((min_,max_))))
    # idx = 0
    # # print(-math.atan2(end_point1[0] - point[0],end_point1[1]- point[1])*(180/math.pi)+90,int(-math.atan2(end_point1[0] - point[0],end_point1[1]- point[1])*(180/math.pi)+90))
    # # print(math.atan2(end_point1[1]- point[1],end_point1[0] - point[0])*(180/math.pi),int(-math.atan2(end_point1[0] - point[0],end_point1[1]- point[1])*(180/math.pi)+90))
    # # print(-math.atan2(end_point2[0] - point[0],end_point2[1]- point[1])*(180/math.pi)+90,int(-math.atan2(end_point2[0] - point[0],end_point2[1]- point[1])*(180/math.pi)+90))    
    
    # while int(math.atan2(end_point1[1]- point[1],end_point1[0] - point[0])*(180/math.pi)) not in range(*sorted((min_,max_))) or int(math.atan2(end_point2[1]- point[1],end_point2[0] - point[0])*(180/math.pi)) not in range(*sorted((min_,max_))):
        # radiuses[i] = radiuses[i]+radius_increase
        # end_point1 = np.array([radiuses[i]*math.sin(change+math.atan2(edge_point[0],edge_point[1])),radiuses[i]*math.cos(change+math.atan2(edge_point[0],edge_point[1]))])
        # end_point2 = np.array([radiuses[i]*math.sin(-change+math.atan2(edge_point[0],edge_point[1])),radiuses[i]*math.cos(-change+math.atan2(edge_point[0],edge_point[1]))]) 
        # idx = idx+1
        # if idx > 10000:
            # break
    # # print(idx)
    
    # # print(-math.atan2(end_point1[0] - point[0],end_point1[1]- point[1])*(180/math.pi)+90,int(-math.atan2(end_point1[0] - point[0],end_point1[1]- point[1])*(180/math.pi)+90))
    # # print(-math.atan2(end_point2[0] - point[0],end_point2[1]- point[1])*(180/math.pi)+90,int(-math.atan2(end_point2[0] - point[0],end_point2[1]- point[1])*(180/math.pi)+90))
    
# # print(math.atan2(end_point1[0],end_point1[1])*(180/math.pi))
    # # print(math.atan2(point[0],point[1])*(180/math.pi))
    # # print(math.atan2(end_point2[0],end_point2[1])*(180/math.pi))
    # # print(((math.atan2(point[0],point[1]))-(math.atan2(end_point2[0],end_point2[1])))*(180/math.pi))
    # # print(((math.atan2(point[0],point[1]))-(math.atan2(end_point1[0],end_point1[1])))*(180/math.pi))
    # # print(((math.atan2(end_point1[0],end_point1[1]))-(math.atan2(end_point2[0],end_point2[1])))*(180/math.pi))
        
    # # ===============Graph section ==============================

    # draw_circle = plt.Circle((0,0),radius,fill=False,edgecolor = "black")
    # plt.gcf().gca().add_artist(draw_circle)

    # plt.xlim(xmin=-max(radius+100,250),xmax=max(radius+100,250))
    # plt.ylim(ymin=-max(radius+100,250),ymax=max(radius+100,250))
    # draw_arc = pth.Arc((0,0),radiuses[i]*2,radiuses[i]*2,min(-(math.atan2(end_point2[0],end_point2[1])*(180/math.pi)-90),-(math.atan2(end_point1[0],end_point1[1])*(180/math.pi)-90)),0,distance,fill=False,edgecolor = "red")
    # plt.gcf().gca().add_artist(draw_arc)
    # plt.plot(0,0,marker="x", markersize=20, markerfacecolor="red", markeredgecolor = "red")
    # m= 0
    # for n in edge_dict:
        # point_start = np.array([math.sin(-(n[0]*(math.pi/180)-(math.pi/2)))*100,math.cos(-(n[0]*(math.pi/180)-(math.pi/2)))*100])
        # point_end = np.array([math.sin(-(n[1]*(math.pi/180)-(math.pi/2)))*100,math.cos(-(n[1]*(math.pi/180)-(math.pi/2)))*100])
        # plt.plot((point_start[0]+leg_Positions[m,0],leg_Positions[m,0]),(point_start[1]+leg_Positions[m,1],leg_Positions[m,1]),'k-',lw=3)
        # plt.plot((point_end[0]+leg_Positions[m,0],leg_Positions[m,0]),(point_end[1]+leg_Positions[m,1],leg_Positions[m,1]),'k-',lw=3)
        # m = m+1
        
        
    # m= 0
     
    # for n in leg_Positions:
        # if m == i:
            # plt.plot((point_mid[0]+leg_Positions[m,0],leg_Positions[m,0]),(point_mid[1]+leg_Positions[m,1],leg_Positions[m,1]),'b-',lw=2)
            # plt.plot(n[0],n[1],marker=".", markersize=20, markerfacecolor="blue", markeredgecolor = "blue")
        # else:
            # plt.plot(n[0],n[1],marker=".", markersize=20, markerfacecolor="black", markeredgecolor = "black")
        # m = m+1
        
    # # plt.plot(point1[0],point1[1],marker="o", markersize=3, markerfacecolor="red",markeredgecolor = "red")
    # # plt.plot(point2[0],point2[1],marker="o", markersize=3, markerfacecolor="red",markeredgecolor = "red")
    # # plt.plot((point1[0],point2[0]),(point1[1],point2[1]),'r-',lw=2    )
    # plt.plot(edge_point[0],edge_point[1],marker="o", markersize=8, markerfacecolor="red",markeredgecolor = "red")
    
    # if speed > 0:
        # start_point = end_point2
        # end_point = end_point1
    # else:
        # start_point = end_point1
        # end_point = end_point2
        
    # plt.plot(end_point[0],end_point[1],marker=".", markersize=8, markerfacecolor="cyan",markeredgecolor = "cyan")
    # plt.plot(start_point[0],start_point[1],marker="*", markersize=8, markerfacecolor="cyan",markeredgecolor = "cyan")    
    # plt.show()
        
    # print("start",start_point)
    # print("end",end_point)
    
# print(radiuses)

















