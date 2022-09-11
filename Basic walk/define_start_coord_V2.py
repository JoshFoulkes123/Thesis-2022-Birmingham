import numpy as np
import math
from matplotlib import pyplot as plt

print("started")
#hyperparams

#functions
edge_angles = np.array([[-135,45],[-180,90]])

def divergence(P1,P2,a):
    out  = abs(((P2[0]-P1[0])*(P1[1]-a[1])-(P1[1]-a[0])*(P2[1]-P1[1]))/(math.sqrt((P2[0]-P1[0])**2+(P2[1]-P1[1])**2)))
    return out

def define_start_stop_coords(want_stop,coord,distance,angle,start_edge_angle,end_edge_angle,leg_index):

    # a = converter.translate_coord(coord.reshape((3,1)),leg_index,1)
    # val = control.IK(a)

        
        
    start_coord = np.array([[0],[0],[0]])
    start_coord[2] = coord[2]
    end_coord = np.array([[0],[0],[0]])
    end_coord[2] = coord[2]
    #go from coord in direction of angle for distance  
    if want_stop == 1:
        start_coord = coord
        end_coord[0] = coord[0]+math.sin(angle)*distance
        end_coord[1] = coord[1]+math.cos(angle)*distance
    
    else:
        end_coord = coord
        start_coord[0] = coord[0]-math.sin(angle)*distance
        start_coord[1] = coord[1]-math.cos(angle)*distance


    #plt.plot(end_coord[0],end_coord[1],marker="x", markersize=20, markerfacecolor="blue")
    #plt.plot(start_coord[0],start_coord[1],marker="x", markersize=20, markerfacecolor="red")
    #plt.arrow(float(start_coord[0]),float(start_coord[1]),float(end_coord[0]-start_coord[0]),float(end_coord[1]-start_coord[1]),width = 0.05,length_includes_head = True)
    #check if valid
    ##check IK
    # a = converter.translate_coord(coord.reshape((3,1)),leg_index,1)
    # val = control.IK(a)
    print(start_coord)
    print(end_coord)
    
    point_start = np.array([math.sin(start_edge_angle*(math.pi/180))*100,math.cos(start_edge_angle*(math.pi/180))*100])
    point_end = np.array([math.sin(end_edge_angle*(math.pi/180))*100,math.cos(end_edge_angle*(math.pi/180))*100])
    

    min = start_edge_angle*(math.pi/180)
    max = end_edge_angle*(math.pi/180)
    if int(round(math.atan2(end_coord[0],end_coord[1]))*(180/math.pi)) not in range(*sorted((int(min*(180/math.pi)),int(max*(180/math.pi))))):

        
        point_s_ = np.array([math.sin(start_edge_angle*(math.pi/180))*1,math.cos(start_edge_angle*(math.pi/180))*1])    
        point_s = np.array([point_s_[0]+math.sin(angle)*distance,point_s_[1]+math.cos(angle)*distance]) 
        
        point_s2 = np.array([math.sin(start_edge_angle*(math.pi/180))*2,math.cos(start_edge_angle*(math.pi/180))*2])
        point_s2 = np.array([point_s2[0]+math.sin(angle)*distance,point_s2[1]+math.cos(angle)*distance])

        
        point_e_ = np.array([math.sin(end_edge_angle*(math.pi/180))*1,math.cos(end_edge_angle*(math.pi/180))*1])
        point_e = np.array([point_e_[0]+math.sin(angle)*distance,point_e_[1]+math.cos(angle)*distance])

        point_e2 = np.array([math.sin(end_edge_angle*(math.pi/180))*2,math.cos(end_edge_angle*(math.pi/180))*2])
        point_e2 = np.array([point_e2[0]+math.sin(angle)*distance,point_e2[1]+math.cos(angle)*distance])
        
        ang = (start_edge_angle-end_edge_angle)/2+end_edge_angle
        point_mid = np.array([math.sin(ang*(math.pi/180))*100,math.cos(ang*(math.pi/180))*100])
        
        div_diff_s = divergence(np.array([0,0]),point_mid,point_s2)-divergence(np.array([0,0]),point_mid,point_s)
        div_diff_e = divergence(np.array([0,0]),point_mid,point_e2)-divergence(np.array([0,0]),point_mid,point_e)
        
        

        if div_diff_e < div_diff_s:
            selected =np.array([math.sin(end_edge_angle*(math.pi/180))*1,math.cos(end_edge_angle*(math.pi/180))*1,0])  
            end_coord = np.append(point_e,coord[2])
            start_coord = np.append(point_e_,coord[2])
        else:
            selected = np.array([math.sin(start_edge_angle*(math.pi/180))*1,math.cos(start_edge_angle*(math.pi/180))*1,0])
            end_coord = np.append(point_s,coord[2]) 
            start_coord = np.append(point_s_,coord[2]) 
        idx = 0
        
      
      
        while int(round(math.atan2(end_coord[0],end_coord[1])*(180/math.pi))) not in range(*sorted((int(min*(180/math.pi)),int(max*(180/math.pi))))):

            if idx > 10000:
                break
            end_coord = end_coord + selected
            start_coord = start_coord + selected
            idx = idx+1

    
        #print(idx)
        
    else:
        print("valid")

    ##check angles
    #if so return start coords


    
    
    #plt.grid()
    #plt.plot(end_coord[0],end_coord[1],marker="x", markersize=20, markerfacecolor="green")
    #plt.plot(start_coord[0],start_coord[1],marker="x", markersize=20, markerfacecolor="yellow")
    #plt.arrow(float(start_coord[0]),float(start_coord[1]),float(end_coord[0]-start_coord[0]),float(end_coord[1]-start_coord[1]),width = 0.05,length_includes_head = True)
    #plt.plot((point_start[0],0),(point_start[1],0),'k-',lw=3)
    #plt.plot((point_end[0],0),(point_end[1],0),'b-',lw=3)
    #plt.plot((point_mid[0],0),(point_mid[1],0),'b.',lw=3)
    
    #plt.show()
    
    return(start_coord.reshape((3,1)),end_coord.reshape((3,1)))



#main
print("Hello world")
print(edge_angles[0][1])
print(edge_angles[1][1])
coords,coords2 = define_start_stop_coords(0,np.array([[2],[5],[3]]),100,-0.3,edge_angles[0][0],edge_angles[1][0],0)

print(coords,coords2)

