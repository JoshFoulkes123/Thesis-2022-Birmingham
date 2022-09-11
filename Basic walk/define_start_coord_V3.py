import numpy as np
import math
from matplotlib import pyplot as plt


print("hello world")

_speed = 0.3 #degrees per iteration

leg_Positions  = np.array([[115,-70,0],[55,-140,0],[-55,-140,0],[-115,-70,0],[115,70,0],[55,140,0],[-55,140,0],[-115,70,0]])

edge_dict = np.array([[-45,0],[-90,-45],[-135,-90],[-180,-135],[45,0],[90,45],[135,90],[180,135]])

walk_height = -250

inital_looking = 30

set_pos = np.array([[100,-100,walk_height],
                    [100,-200,walk_height],
                    [-100,-200,walk_height],
                    [-200,-100,walk_height],
                    [200,100,walk_height],
                    [100,200,walk_height],
                    [-100,200,walk_height],
                    [-200,100,walk_height]])

def divergence(P1,P2,a):
    out  = abs(((P2[0]-P1[0])*(P1[1]-a[1])-(P1[1]-a[0])*(P2[1]-P1[1]))/(math.sqrt((P2[0]-P1[0])**2+(P2[1]-P1[1])**2)))
    return out
   
def define_start_stop_coordsV3(want_stop,distance,angle,leg_index):
   

    mid_angle = (edge_dict[leg_index,0]-edge_dict[leg_index,1])/2+edge_dict[leg_index,1]
    point_mid = np.array([math.sin(-(mid_angle*(math.pi/180)-(math.pi/2)))*100,math.cos(-(mid_angle*(math.pi/180)-(math.pi/2)))*100])
    print(mid_angle)

    initial_point = np.array([int(set_pos[leg_index,0]),int(set_pos[leg_index,1])])
    
    flipped= 0       
        
    if angle > 180 :
        new_ang = angle -360
    elif angle < -180:
        new_ang = angle +360
    else: 
        new_ang = angle

    if leg_index == 0 or leg_index == 4:
        if new_ang > 90 or new_ang < -90:
            new_ang= new_ang -180
            flipped = 1           
    elif leg_index == 1 or leg_index == 2:
        if new_ang > 0 or new_ang < -180:
            new_ang= new_ang -180
            flipped = 1     
    elif leg_index == 5 or leg_index == 6:
        if new_ang > 180 or new_ang < 0:
            new_ang= new_ang -180
            flipped = 1    
    else:
        if new_ang > -90 and new_ang < 90:
            new_ang= new_ang -180
            flipped = 1   
        
    end_point = initial_point + np.array([math.cos(new_ang*(math.pi/180))*distance,math.sin(new_ang*(math.pi/180))*distance])
    print(initial_point,end_point)

    _min = edge_dict[leg_index,1]
    _max = edge_dict[leg_index,0]    
    print(range(*sorted((int(_min),int(_max)))))

    
    print(math.atan2((end_point[1])-leg_Positions[leg_index,1],(end_point[0])-leg_Positions[leg_index,0])*(180/math.pi))
    print(math.atan2((initial_point[1])-leg_Positions[leg_index,1],(initial_point[0])-leg_Positions[leg_index,0])*(180/math.pi))

    

    if int(math.atan2((end_point[1])-leg_Positions[leg_index,1],(end_point[0])-leg_Positions[leg_index,0])*(180/math.pi)) not in range(*sorted((int(_min),int(_max)))) or  int(math.atan2((initial_point[1])-leg_Positions[leg_index,1],(initial_point[0])-leg_Positions[leg_index,0])*(180/math.pi)) not in range(*sorted((int(_min),int(_max)))):
        print("mark")
        
      
        start_edge_angle = edge_dict[leg_index,1]
        print(start_edge_angle)
        end_edge_angle = edge_dict[leg_index,0]
                
        
        start_coord = np.array([leg_Positions[leg_index,0]+math.cos(mid_angle*(math.pi/180))*inital_looking,leg_Positions[leg_index,1]+math.sin(mid_angle*(math.pi/180))*inital_looking]) 
        end_coord = start_coord+np.array([math.cos(new_ang*(math.pi/180))*distance,math.sin(new_ang*(math.pi/180))*distance])
        # plt.plot(start_coord[0],start_coord[1],marker="o", markersize=7, markerfacecolor="none", markeredgecolor = "blue")
        # plt.plot(end_coord[0],end_coord[1],marker="o", markersize=7, markerfacecolor="none", markeredgecolor = "green")
        
        point_1_e1_s = start_coord+np.array([math.cos(start_edge_angle*(math.pi/180))*1,math.sin(start_edge_angle*(math.pi/180))*1])
        point_1_e1_e = point_1_e1_s+np.array([math.cos(new_ang*(math.pi/180))*distance,math.sin(new_ang*(math.pi/180))*distance])
        point_2_e1_s = start_coord+np.array([math.cos(start_edge_angle*(math.pi/180))*2,math.sin(start_edge_angle*(math.pi/180))*2])
        point_2_e1_e = point_2_e1_s+np.array([math.cos(new_ang*(math.pi/180))*distance,math.sin(new_ang*(math.pi/180))*distance])
        
        # plt.plot(point_1_e1_s[0],point_1_e1_s[1],marker="o", markersize=7, markerfacecolor="none", markeredgecolor = "blue")
        # plt.plot(point_2_e1_s[0],point_2_e1_s[1],marker="o", markersize=7, markerfacecolor="none", markeredgecolor = "green")
        # plt.plot(point_1_e1_e[0],point_1_e1_e[1],marker="o", markersize=7, markerfacecolor="none", markeredgecolor = "blue")
        # plt.plot(point_2_e1_e[0],point_2_e1_e[1],marker="o", markersize=7, markerfacecolor="none", markeredgecolor = "green")
        
        point_1_s1_s = start_coord+np.array([math.cos(end_edge_angle*(math.pi/180))*1,math.sin(end_edge_angle*(math.pi/180))*1])
        point_1_s1_e = point_1_s1_s+np.array([math.cos(new_ang*(math.pi/180))*distance,math.sin(new_ang*(math.pi/180))*distance])
        point_2_s1_s = start_coord+np.array([math.cos(end_edge_angle*(math.pi/180))*2,math.sin(end_edge_angle*(math.pi/180))*2])
        point_2_s1_e = point_2_s1_s+np.array([math.cos(new_ang*(math.pi/180))*distance,math.sin(new_ang*(math.pi/180))*distance])
        
        # plt.plot(point_1_s1_s[0],point_1_s1_s[1],marker="s", markersize=7, markerfacecolor="none", markeredgecolor = "blue")
        # plt.plot(point_2_s1_s[0],point_2_s1_s[1],marker="s", markersize=7, markerfacecolor="none", markeredgecolor = "green")
        # plt.plot(point_1_s1_e[0],point_1_s1_e[1],marker="s", markersize=7, markerfacecolor="none", markeredgecolor = "blue")
        # plt.plot(point_2_s1_e[0],point_2_s1_e[1],marker="s", markersize=7, markerfacecolor="none", markeredgecolor = "green")
        
        div_diff_s = divergence(np.array([0,0]),point_mid,point_2_s1_e)-divergence(np.array([0,0]),point_mid, point_1_s1_e)
        div_diff_e = divergence(np.array([0,0]),point_mid,point_2_e1_e)-divergence(np.array([0,0]),point_mid,point_1_e1_e)
        
        print(div_diff_s,div_diff_e)
        print(start_edge_angle,end_edge_angle)
        
        if div_diff_e > div_diff_s:
            selected =np.array([math.cos(end_edge_angle*(math.pi/180))*1,math.sin(end_edge_angle*(math.pi/180))*1,0])  
            print(end_edge_angle)
            end_coord = np.append(point_1_e1_e,walk_height)
            start_coord = np.append(point_1_e1_s,walk_height)
        else:
            selected = np.array([math.cos(start_edge_angle*(math.pi/180))*1,math.sin(start_edge_angle*(math.pi/180))*1,0])
            print(start_edge_angle)
            end_coord = np.append(point_1_s1_e,walk_height)
            start_coord = np.append(point_1_s1_s,walk_height) 
            
        print(selected)
        
        idx = 0
        print(range(*sorted((int(_min),int(_max)))))
        while int(math.atan2((end_coord[1])-leg_Positions[leg_index,1],(end_coord[0])-leg_Positions[leg_index,0])*(180/math.pi)) not in range(*sorted((int(_min),int(_max)))):

            if idx > 1000:
                break
            #print(int(math.atan2((end_coord[1])-leg_Positions[leg_index,1],(end_coord[0])-leg_Positions[leg_index,0])*(180/math.pi)))
            end_coord = end_coord + selected
            start_coord = start_coord + selected
            idx = idx+1    
    else:
        end_point = np.append(end_point,walk_height)
        initial_point = np.append(initial_point,walk_height)
    
    try: 
        initial_point = start_coord
        end_point = end_coord
    except:
        pass    
    
    


    
    
    
    #graph section =======================================================
    plt.plot(0,0,marker="x", markersize=20, markerfacecolor="black", markeredgecolor = "black")
    m= 0
    for n in edge_dict:
        point_start = np.array([math.sin(-(n[0]*(math.pi/180)-(math.pi/2)))*100,math.cos(-(n[0]*(math.pi/180)-(math.pi/2)))*100])
        point_end = np.array([math.sin(-(n[1]*(math.pi/180)-(math.pi/2)))*100,math.cos(-(n[1]*(math.pi/180)-(math.pi/2)))*100])
        plt.plot((point_start[0]+leg_Positions[m,0],leg_Positions[m,0]),(point_start[1]+leg_Positions[m,1],leg_Positions[m,1]),'k-',lw=3)
        plt.plot((point_end[0]+leg_Positions[m,0],leg_Positions[m,0]),(point_end[1]+leg_Positions[m,1],leg_Positions[m,1]),'k-',lw=3)
        m = m+1
        
        
    m= 0
     
    for n in leg_Positions:
        print(n)
        if m == 0:
            plt.plot((point_mid[0]+leg_Positions[m,0],leg_Positions[m,0]),(point_mid[1]+leg_Positions[m,1],leg_Positions[m,1]),'b-',lw=2)
            plt.plot(n[0],n[1],marker=".", markersize=20, markerfacecolor="blue", markeredgecolor = "blue")
        else:
            plt.plot(n[0],n[1],marker=".", markersize=20, markerfacecolor="black", markeredgecolor = "black")
        m = m+1
       
    plt.arrow(0,0,100,0,width = 7,length_includes_head = True,color= "red") 
    plt.arrow(0,0,0,100,width = 7,length_includes_head = True, color = "green") 
  
    plt.plot(initial_point[0],initial_point[1],marker=".", markersize=10, markerfacecolor="red", markeredgecolor = "red")
    
    
    plt.plot(set_pos[0,0],set_pos[0,1],marker=".", markersize=10, markerfacecolor="red", markeredgecolor = "green")
    
    try:
        plt.plot(start_coord[0],start_coord[1],marker="o", markersize=7, markerfacecolor="none", markeredgecolor = "red")
        plt.plot(end_coord[0],end_coord[1],marker="o", markersize=7, markerfacecolor="none", markeredgecolor = "black")
    except:
        pass
    
    if flipped == 1:
        plt.arrow(end_point[0],end_point[1],-(end_point[0]-initial_point[0]),-(end_point[1]-initial_point[1]),width = 5,length_includes_head = True) 
        plt.plot(end_point[0],end_point[1],marker="+", markersize=20, markerfacecolor="black", markeredgecolor = "black")
    else:
        plt.arrow(initial_point[0],initial_point[1],end_point[0]-initial_point[0],end_point[1]-initial_point[1],width = 5,length_includes_head = True)
        plt.plot(end_point[0],end_point[1],marker="+", markersize=20, markerfacecolor="black", markeredgecolor = "black")        
        
    plt.show()  
    
    print(end_point,initial_point)
    print(flipped)
    if flipped == 1:
        return end_point.reshape(3,1),initial_point.reshape(3,1)
    else:
        return initial_point.reshape(3,1),end_point.reshape(3,1)
    
    
for i in range(8):

    print("==========================="+str(i+1)+"=========================")
  
    out,out1 = define_start_stop_coordsV3(0,_speed*300,90,i)
    print("out")
    print(out,out1)
    # plt.plot(out[0][0],out[0][1],marker="o", markersize=10, markerfacecolor="none", markeredgecolor = "red")
    # plt.plot(out[1][0],out[1][1],marker="o", markersize=10, markerfacecolor="none", markeredgecolor = "blue")
    # plt.show()
    
