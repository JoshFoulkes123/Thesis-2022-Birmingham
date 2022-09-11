"""steve_controller_ea controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
#from controller import Robot
from controller import Robot,Motor,PositionSensor,Supervisor
from controller import Node
from matplotlib import pyplot as plt
from matplotlib import patches as pth
import sys
import warnings

if not sys.warnoptions:
    warnings.simplefilter("ignore")

import math
import numpy as np
import random
from scipy.stats import truncnorm, norm

#intialisation
# create the Robot instance.
#robot = Robot()

# get the time step of the current world.
#timestep = int(robot.getBasicTimeStep())

#representation


representation_basic =            np.array([0.3,      50,        300,       1,         -250,       200,       1,         40,        0,         45,        90,        135,       180,       225,       270,       315,       200,       100,       100,       200,       100,       200,       200,       100,       200,       100,       100,       200,       100,       200,       200,       100,       80])

representation_basic_trained =    np.array([1.133337256938467652e+00, 5.000000000000000000e+01, 2.530000000000000000e+02, 1.062277944192802437e+02, -2.500000000000000000e+02, 7.960000000000000000e+02, 4.801596557658157849e+00, 4.000000000000000000e+01, 1.800000000000000000e+02, 4.500000000000000000e+01, 1.020000000000000000e+02, 1.350000000000000000e+02, 2.000000000000000000e+01, 1.120000000000000000e+02, 1.920000000000000000e+02, 3.700000000000000000e+01, 2.000000000000000000e+02, 1.770000000000000000e+02, 1.000000000000000000e+02, 2.000000000000000000e+02, 1.000000000000000000e+02, 2.710000000000000000e+02, 2.000000000000000000e+02, 5.900000000000000000e+01, 7.000000000000000000e+01, 9.300000000000000000e+01, 1.000000000000000000e+02, 3.720000000000000000e+02, 1.000000000000000000e+02, 2.000000000000000000e+02, 2.000000000000000000e+02, 1.000000000000000000e+02, 5.880000000000000000e+02])

representation_improved =         np.array([0.6,      50,        300,       1,         -250,       200,       1,         180,        0,         180,        0,         180,        180,        0,         180,        0,         200,       100,       100,       200,       100,       200,       200,       100,       200,       100,       100,       200,       100,       200,       200,       100,       80])

representation_improved_trained = np.array([1.945462558111680895e+00, 7.206708932058103301e+01, 3.600000000000000000e+01, 8.342476919027211579e+01, -2.500000000000000000e+02, 6.090000000000000000e+02, 9.553728455399379982e+00, 9.500000000000000000e+01, 0, 9.500000000000000000e+01, 0, 9.500000000000000000e+01, 9.500000000000000000e+01, 0,9.500000000000000000e+01, 0, 3.080000000000000000e+02, 1.000000000000000000e+02, 6.600000000000000000e+01, 2.000000000000000000e+02, 8.600000000000000000e+01, 1.560000000000000000e+02, 2.040000000000000000e+02, 1.170000000000000000e+02, 3.190000000000000000e+02, 1.000000000000000000e+02, 1.000000000000000000e+02, 2.000000000000000000e+02, 2.070000000000000000e+02, 2.890000000000000000e+02, 2.000000000000000000e+02, 1.310000000000000000e+02, 3.830000000000000000e+02])

if np.size(representation_basic_trained,axis = 0) != np.size(representation_basic,axis = 0):

    raise ValueError("unequal sizes")
    
if np.size(representation_improved_trained,axis = 0) != np.size(representation_improved,axis = 0):

    raise ValueError("unequal sizes")



#Hyperpartmaetrs
##max iteration

fitness_time = 5


##===================================controller=======================================
#forwards
spider_angle = 0
#turn
turn = 0 #1 means turning
turn_speed = 0.07
#================================parameters to be trained==============================
inital_looking = 30

spider_speed = 0

#starting distance from the radius to search for valid turns
mid_start_coord = 50

#amount of time leg is on ground when turning
ground_turn_time = 300

#radius increase of serach when point is valid for rotation
radius_increase = 1

#the walking height of the robot
walk_height = -250

#min searching radius for turn
radius = 200

#ratio between width of ellipse path and height
HV_ratio = 1

#time in the air of leg
swing_time = 40

#start of each legs swing section
walking_starts = np.array([0,45,90,135,180,225,270,315])

#total units in cycle
len_of_cycle = int(max(walking_starts)+swing_time+1)

#setup init pose
hips = [-36,-72,-108,-144,36,72,108,144]
legs = [40,40]

#intial search postions for leg
set_pos = np.array([[200,-100,walk_height],
                    [100,-200,walk_height],
                    [-100,-200,walk_height],
                    [-200,-100,walk_height],
                    
                    [200,100,walk_height],
                    [100,200,walk_height],
                    [-100,200,walk_height],
                    [-200,100,walk_height]])

# ================================functions ====================================
#forwards
spider_speed = 0
spider_angle = 0
#turn
turn = 0 #1 means turning
turn_speed = 0.07

#================================setup=======================

# create the Robot instance.
robot = Supervisor()
node = robot.getSelf()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


#Leg order: 1R,2R,3R,4R,1L,2L,3L,4L
#angles of each leg
leg_Angles = np.array([[-0.698134],[-1.22173],[-1.91986],[-2.44346],[0.698134],[1.22173],[1.91986],[2.44346]])
#xy position of each leg
leg_Positions  = np.array([[115,-70,0],[55,-140,0],[-55,-140,0],[-115,-70,0],[115,70,0],[55,140,0],[-55,140,0],[-115,70,0]])
#max and min of each leg
edge_dict = np.array([[-45,0],[-90,-45],[-135,-90],[-180,-135],[45,0],[90,45],[135,90],[180,135]])


#----------------coordinate converter---------------------
class coord_conversion:
    def __init__(self,angs,pos):
        self.positions = pos
        self.angles = angs
        self.rotations = self.rotZ(angs[0])
        self.rotations = self.rotations[:,:,np.newaxis]
        self.angles = np.delete(angs,0)
        for ang in self.angles:
            val = self.rotZ(ang)
            val = val[:,:,np.newaxis]
            self.rotations =np.concatenate((self.rotations,val),2)
        
        self.angles= angs
        
    def translate_coord(self,coords,index,forward_back):
        #1 means from leg to coord output p1
        #0 means from body to coord output p0
        #switch x and y as code refuses to do it
        R = self.rotations[:,:,index]
        b = self.positions[index,:]

        if forward_back == 1:
            out = -np.matmul(np.transpose(R),b.reshape(3,1))+(np.matmul(np.transpose(R),coords.reshape(3,1)))
        else:
            b.reshape(3,1)
            coords.reshape(3,1)
            out = (b.reshape(3,1)) + (np.matmul(R,coords.reshape(3,1)))
        return out
       
        
    def convert_angle(self,angle,index):
        out = (self.angles[index])+angle
    
        return out
        
    def rotZ(self,ang):
        out = np.array([[math.cos(ang),-math.sin(ang),0],[math.sin(ang),math.cos(ang),0],[0,0,1]])
        return out
        
        
#------------------------controller----------------------------

class Joint:
    def __init__(self,position,parent_name):
        switcher = {
            0: "base_link_fixed_hip2",
            1: "hip2_hipjoint",
            2: "uknee_lknee",
        }
        pos = switcher.get(position, "nothing")
        self.actuator =robot.getDevice(str(parent_name+"_"+pos))
        self.position_sensor = robot.getDevice(str(parent_name+"_"+pos+"_sensor"))
        self.position_sensor.enable(timestep)
        
class Leg:
    def __init__(self,name):
        self.name = name
        self.top  = Joint(0,self.name)
        self.mid  = Joint(1,self.name)
        self.bottom  = Joint(2,self.name)
        self.touch_sensor = robot.getDevice(str("touch_sensor_"+name))
        self.touch_sensor.enable(timestep)
    def move(self,angles):
        self.top.actuator.setPosition(float(angles[0]))
        self.mid.actuator.setPosition(float(angles[1]))
        self.bottom.actuator.setPosition(float(angles[2]))
    def read(self):
        out = np.array([[0],[0],[0],[0]],dtype='f')
        out[0]=self.top.position_sensor.getValue()
        out[1]=self.mid.position_sensor.getValue()
        out[2]=self.bottom.position_sensor.getValue()
        out[3]=self.touch_sensor.getValue()
        return out
    def read_Torques(self):
        out = np.array([[0],[0],[0],[0]],dtype='f')
        out[0]=self.top.actuator.getTorqueFeedback()
        out[1]=self.mid.actuator.getTorqueFeedback()
        out[2]=self.bottom.actuator.getTorqueFeedback()

        return out
  
#------------------------main controller----------------------------  
 
class Leg_controller:
    def __init__(self):
        self.leg_1R = Leg("leg1R")
        self.leg_2R = Leg("leg2R")
        self.leg_3R = Leg("leg3R")
        self.leg_4R = Leg("leg4R")
        self.leg_1L = Leg("leg1L")
        self.leg_2L = Leg("leg2L")
        self.leg_3L = Leg("leg3L")
        self.leg_4L = Leg("leg4L")
        self.gyro = robot.getDevice("gyro")
        self.gyro.enable(timestep)
       
       

    def get_leg(self,leg_name):
        lib ={  
        "leg_1R" : self.leg_1R,         
        "leg_2R" : self.leg_2R,        
        "leg_3R" : self.leg_3R,         
        "leg_4R" : self.leg_4R, 
        
        "leg_1L" : self.leg_1L,        
        "leg_2L" : self.leg_2L,       
        "leg_3L" : self.leg_3L,     
        "leg_4L" : self.leg_4L,         
        }
        
        out = lib.get(leg_name, -1)
        
        return out
        
    def get_gyro(self):
        out = self.gyro.getValues()
        
        return out
    
    def FK(self,q0,q1,q2):
    
        DH_params = np.array([[55,-math.pi/2,0,q0],[130,0,0,q1],[200,0,0,q2]],dtype=object)
        out = np.identity(4)

        for i in range(3):
            curr_ai = DH_params[i,0];
            curr_vi = DH_params[i,3];
            curr_alphai = DH_params[i,1];
            curr_di = DH_params[i,2];
            

            
            curr_hom = np.zeros([4,4])

            curr_hom[0,0]  = math.cos(curr_vi);
            curr_hom[1,0] = math.sin(curr_vi);
            
            curr_hom[0,1] = -math.sin(curr_vi)*math.cos(curr_alphai);
            curr_hom[1,1] = math.cos(curr_vi)*math.cos(curr_alphai);
            curr_hom[2,1] = math.sin(curr_alphai);

            curr_hom[0,2] = math.sin(curr_vi)*math.sin(curr_alphai);
            curr_hom[1,2] = -math.cos(curr_vi)*math.sin(curr_alphai);
            curr_hom[2,2] = math.cos(curr_alphai);

            curr_hom[0,3] = curr_ai*math.cos(curr_vi);
            curr_hom[1,3] = curr_ai*math.sin(curr_vi);
            curr_hom[2,3] = curr_di;
            curr_hom[3,3] = 1;
            

        
            out = np.matmul(out,curr_hom);

        x = int(out[0,3])
        y=int(out[1,3])
        z=int(out[2,3])
        return np.transpose(out[0:3,3]),x,y,z
        
    def IK(self,pos):
        try:
            x = pos[1]
            y = pos[0]
            z = pos[2]
            
            l1 = 55
            l2 = 130
            l3 = 200
        
            x = x +0.0000000001
            y = y +0.0000000001
            z = z +0.0000000001
            L = math.sqrt(y**2+x**2)
            Lt = math.sqrt((L-l1)**2+z**2)
            
            landa = (math.atan((L-l1)/-z))*(180/math.pi)
            beta = (math.acos((l3**2-l2**2-Lt**2)/(-2*l2*Lt)))*(180/math.pi)
            alpha = (math.acos((Lt**2-l2**2-l3**2)/(-2*l2*l3)))*(180/math.pi)
            
            q3 = 180-alpha
            q2 = -(beta-(90-landa))
            q1 = (math.atan(x/y)*(180/math.pi))
        except:
            q3=q2=q1 =404
        
        out = np.array([[round(q1,1)],[round(q2,1)],[round(q3,1)]])
        
        return out
          
#calling previously made classes          
converter = coord_conversion(leg_Angles,leg_Positions)
control = Leg_controller()

#------------------------------staright coordinate creator----------------------- 
def divergence(P1,P2,a):
    out  = abs(((P2[0]-P1[0])*(P1[1]-a[1])-(P1[1]-a[0])*(P2[1]-P1[1]))/(math.sqrt((P2[0]-P1[0])**2+(P2[1]-P1[1])**2)))
    return out

def define_start_stop_coords(want_stop,coord,distance,angle,start_edge_angle,end_edge_angle,leg_index):
    leg_pos = leg_Positions[leg_index,:]
    #plt.plot(leg_pos[0],leg_pos[1],marker="s", markersize=8, markerfacecolor="blue")

    
    start_coord = np.array([[0],[0],[0]])
    start_coord[2] = coord[2]
    end_coord = np.array([[0],[0],[0]])
    end_coord[2] = coord[2] 
    if want_stop == 1:
        start_coord = coord
        end_coord[0] = coord[0]+math.cos(angle)*distance
        end_coord[1] = coord[1]+math.sin(angle)*distance
    
    else:
        end_coord = coord
        start_coord[0] = coord[0]-math.cos(angle)*distance
        start_coord[1] = coord[1]-math.sin(angle)*distance


    #plt.plot(end_coord[0],end_coord[1],marker="x", markersize=20, markerfacecolor="blue")
    #plt.plot(start_coord[0],start_coord[1],marker="s", markersize=12, markerfacecolor="red")
    #plt.arrow(float(start_coord[0]),float(start_coord[1]),float(end_coord[0]-start_coord[0]),float(end_coord[1]-start_coord[1]),width = 3,length_includes_head = True)

    
    point_start = np.array([math.cos(start_edge_angle*(math.pi/180))*100+leg_pos[0],math.sin(start_edge_angle*(math.pi/180))*100+leg_pos[1]])
    point_end = np.array([math.cos(end_edge_angle*(math.pi/180))*100+leg_pos[0],math.sin(end_edge_angle*(math.pi/180))*100+leg_pos[1]])
    
    a = converter.translate_coord(start_coord,leg_index,1)
    angles_start  = control.IK(a)
    a = converter.translate_coord(end_coord,leg_index,1)
    angles_end  = control.IK(a)

    min_ = min(start_edge_angle,end_edge_angle)
    max_ = max(start_edge_angle,end_edge_angle)+1

    
    ang = (start_edge_angle-end_edge_angle)/2+end_edge_angle
    point_mid = np.array([math.cos(ang*(math.pi/180))*100+leg_pos[0],math.sin(ang*(math.pi/180))*100+leg_pos[1]])
    point_mid_start = np.array([math.cos(ang*(math.pi/180))*mid_start_coord+leg_pos[0],math.sin(ang*(math.pi/180))*mid_start_coord+leg_pos[1]])
    if (int(math.atan2(end_coord[1],end_coord[0])*(180/math.pi)) not in range(*sorted((int(min_*(180/math.pi)),int(max_*(180/math.pi)))))) or angles_end[0] == 404 or angles_start[0] == 404:
        
        point_s_ = np.array([math.cos(start_edge_angle*(math.pi/180))*1+point_mid_start[0],math.sin(start_edge_angle*(math.pi/180))*1+point_mid_start[1]])
        #plt.plot(point_s_[0],point_s_[1],marker="+", markersize=20, color="red")    
        point_s = np.array([point_s_[0]+math.cos(angle)*distance,point_s_[1]+math.sin(angle)*distance])
        #plt.plot(point_s[0],point_s[1],marker="+", markersize=20, color="red")     
        
        point_s2 = np.array([math.cos(start_edge_angle*(math.pi/180))*2+point_mid_start[0],math.sin(start_edge_angle*(math.pi/180))*2+point_mid_start[1]])
        #plt.plot(point_s2[0],point_s2[1],marker="+", markersize=20, color="red") 
        point_s2 = np.array([point_s2[0]+math.cos(angle)*distance,point_s2[1]+math.sin(angle)*distance])
        #plt.plot(point_s2[0],point_s2[1],marker="+", markersize=20, color="red") 

        
        point_e_ = np.array([math.cos(end_edge_angle*(math.pi/180))*1+point_mid_start[0],math.sin(end_edge_angle*(math.pi/180))*1+point_mid_start[1]])
        #plt.plot(point_e_[0],point_e_[1],marker="+", markersize=20, color="green") 
        point_e = np.array([point_e_[0]+math.cos(angle)*distance,point_e_[1]+math.sin(angle)*distance])
        #plt.plot(point_e[0],point_e[1],marker="+", markersize=20, color="orange") 

        point_e2 = np.array([math.cos(end_edge_angle*(math.pi/180))*2+point_mid_start[0],math.sin(end_edge_angle*(math.pi/180))*2+point_mid_start[1]])
        #plt.plot(point_e2[0],point_e2[1],marker="+", markersize=20, color="green") 
        point_e2 = np.array([point_e2[0]+math.cos(angle)*distance,point_e2[1]+math.sin(angle)*distance])
        #plt.plot(point_e2[0],point_e2[1],marker="+", markersize=20, color="green") 
        
        
        
        div_diff_s = divergence(np.array([0,0]),point_mid,point_s2)-divergence(np.array([0,0]),point_mid,point_s)
        div_diff_e = divergence(np.array([0,0]),point_mid,point_e2)-divergence(np.array([0,0]),point_mid,point_e)
        
        

        if div_diff_e < div_diff_s:
            selected =np.array([math.cos(end_edge_angle*(math.pi/180))*1,math.sin(end_edge_angle*(math.pi/180))*1,0])  
            end_coord = np.append(point_e,coord[2])
            start_coord = np.append(point_e_,coord[2])
        else:
            selected = np.array([math.cos(start_edge_angle*(math.pi/180))*1,math.sin(start_edge_angle*(math.pi/180))*1,0])
            end_coord = np.append(point_s,coord[2]) 
            start_coord = np.append(point_s_,coord[2]) 
        idx = 0
        
        a = converter.translate_coord(start_coord,leg_index,1)
        angles_start  = control.IK(a)
        a = converter.translate_coord(end_coord,leg_index,1)
        angles_end  = control.IK(a)
 
        
        while (int(converter.convert_angle(angles_start[0]*(math.pi/180),leg_index)*(180/math.pi)) not in range(*sorted((int(min_),int(max_))))) or (int(converter.convert_angle(angles_end[0]*(math.pi/180),leg_index)*(180/math.pi)) not in range(*sorted((int(min_),int(max_))))) or angles_end[0] == 404 or angles_start[0] == 404:#add 45 degree sheck

            a = converter.translate_coord(start_coord,leg_index,1)
            angles_start  = control.IK(a)
            a = converter.translate_coord(end_coord,leg_index,1)
            angles_end  = control.IK(a)                  
            if idx > 1000:
                #print("inavlid")
                break
            end_coord = end_coord + selected
            start_coord = start_coord + selected
            idx = idx+1

        
    else:
        #print("valid")
        pass
  
    #-----------graph bit -------------------------- 
    # plt.grid()
    # plt.plot(end_coord[0],end_coord[1],marker="x", markersize=20, markerfacecolor="green")
    # plt.plot(start_coord[0],start_coord[1],marker="o", markersize=12, markerfacecolor="red")
    # plt.arrow(float(start_coord[0]),float(start_coord[1]),float(end_coord[0]-start_coord[0]),float(end_coord[1]-start_coord[1]),width = 3,length_includes_head = True)
    # plt.plot((point_start[0],leg_pos[0]),(point_start[1],leg_pos[1]),'k-',lw=3)
    # plt.plot((point_end[0],leg_pos[0]),(point_end[1],leg_pos[1]),'b-',lw=3)
    # plt.plot((point_mid[0],leg_pos[0]),(point_mid[1],leg_pos[1]),'r-',lw=3)    
    #plt.show()
    
    return(start_coord.reshape((3,1)),end_coord.reshape((3,1)))   


   
def define_start_stop_coordsV3(want_stop,distance,angle,leg_index):
   

    mid_angle = (edge_dict[leg_index,0]-edge_dict[leg_index,1])/2+edge_dict[leg_index,1]
    point_mid = np.array([math.sin(-(mid_angle*(math.pi/180)-(math.pi/2)))*100,math.cos(-(mid_angle*(math.pi/180)-(math.pi/2)))*100])
    # print(mid_angle)

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
    # print(initial_point,end_point)

    _min = edge_dict[leg_index,1]
    _max = edge_dict[leg_index,0]    
    # print(range(*sorted((int(_min),int(_max)))))

    
    # print(math.atan2((end_point[1])-leg_Positions[leg_index,1],(end_point[0])-leg_Positions[leg_index,0])*(180/math.pi))
    # print(math.atan2((initial_point[1])-leg_Positions[leg_index,1],(initial_point[0])-leg_Positions[leg_index,0])*(180/math.pi))

    a = converter.translate_coord(np.append(initial_point,walk_height),leg_index,1)
    angles_start  = control.IK(a)
    a = converter.translate_coord(np.append(end_point,walk_height),leg_index,1)
    angles_end  = control.IK(a)

    if int(math.atan2((end_point[1])-leg_Positions[leg_index,1],(end_point[0])-leg_Positions[leg_index,0])*(180/math.pi)) not in range(*sorted((int(_min),int(_max)))) or  int(math.atan2((initial_point[1])-leg_Positions[leg_index,1],(initial_point[0])-leg_Positions[leg_index,0])*(180/math.pi)) not in range(*sorted((int(_min),int(_max)))) or angles_end[0] == 404 or angles_start[0] == 404:
        # print("mark")
        
      
        start_edge_angle = edge_dict[leg_index,1]
        # print(start_edge_angle)
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
        
        # print(div_diff_s,div_diff_e)
        # print(start_edge_angle,end_edge_angle)
        
        if div_diff_e > div_diff_s:
            selected =np.array([math.cos(end_edge_angle*(math.pi/180))*1,math.sin(end_edge_angle*(math.pi/180))*1,0])  
            # print(end_edge_angle)
            end_coord = np.append(point_1_e1_e,walk_height)
            start_coord = np.append(point_1_e1_s,walk_height)
        else:
            selected = np.array([math.cos(start_edge_angle*(math.pi/180))*1,math.sin(start_edge_angle*(math.pi/180))*1,0])
            # print(start_edge_angle)
            end_coord = np.append(point_1_s1_e,walk_height)
            start_coord = np.append(point_1_s1_s,walk_height) 
            
        # print(selected)
        
        idx = 0
        # print(range(*sorted((int(_min),int(_max)))))
        a = converter.translate_coord(start_coord,leg_index,1)
        angles_start  = control.IK(a)
        a = converter.translate_coord(end_coord,leg_index,1)
        angles_end  = control.IK(a)
        
        
        while int(math.atan2((end_coord[1])-leg_Positions[leg_index,1],(end_coord[0])-leg_Positions[leg_index,0])*(180/math.pi)) not in range(*sorted((int(_min),int(_max)))) or angles_end[0] == 404 or angles_start[0] == 404:
            a = converter.translate_coord(start_coord,leg_index,1)
            angles_start  = control.IK(a)
            a = converter.translate_coord(end_coord,leg_index,1)
            angles_end  = control.IK(a)
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
        # if m == leg_index:
            # plt.plot((point_mid[0]+leg_Positions[m,0],leg_Positions[m,0]),(point_mid[1]+leg_Positions[m,1],leg_Positions[m,1]),'b-',lw=2)
            # plt.plot(n[0],n[1],marker=".", markersize=20, markerfacecolor="blue", markeredgecolor = "blue")
        # else:
            # plt.plot(n[0],n[1],marker=".", markersize=20, markerfacecolor="black", markeredgecolor = "black")
        # m = m+1
        
    # plt.plot(initial_point[0],initial_point[1],marker=".", markersize=10, markerfacecolor="red", markeredgecolor = "red")
    
    # try:
        # plt.plot(start_coord[0],start_coord[1],marker="o", markersize=7, markerfacecolor="none", markeredgecolor = "red")
        # plt.plot(end_coord[0],end_coord[1],marker="o", markersize=7, markerfacecolor="none", markeredgecolor = "black")
    # except:
        # pass
    
    # if flipped == 1:
        # plt.arrow(end_point[0],end_point[1],-(end_point[0]-initial_point[0]),-(end_point[1]-initial_point[1]),width = 5,length_includes_head = True) 
        # plt.plot(end_point[0],end_point[1],marker="+", markersize=20, markerfacecolor="black", markeredgecolor = "black")
    # else:
        # plt.arrow(initial_point[0],initial_point[1],end_point[0]-initial_point[0],end_point[1]-initial_point[1],width = 5,length_includes_head = True)
        # plt.plot(end_point[0],end_point[1],marker="+", markersize=20, markerfacecolor="black", markeredgecolor = "black")        
        
    # plt.show()
    
    
    if flipped == 1:
        return end_point.reshape(3,1),initial_point.reshape(3,1)
    else:
        return initial_point.reshape(3,1),end_point.reshape(3,1)
        
       


def start_stop_turn(radius,Speed,time,leg_index,walk_height):
    speed = abs(Speed)
    i = leg_index
    distance = speed*time
    radiuses = np.ones(8)*radius
    
    angle = (edge_dict[i,0]-edge_dict[i,1])/2+edge_dict[i,1]
    point_mid = np.array([math.sin(-(angle*(math.pi/180)-(math.pi/2)))*100,math.cos(-(angle*(math.pi/180)-(math.pi/2)))*100])


    if angle == 0:
        angle = 180
    angle = (-(angle-90))*(math.pi/180)
    e = leg_Positions
    point = e[i]

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
    idx = 0
    
    end_point1 = np.append(end_point1,walk_height)
    end_point2 = np.append(end_point2,walk_height) 
   
    a = converter.translate_coord(end_point1,leg_index,1)
    angles_start  = control.IK(a)
    a = converter.translate_coord(end_point2,leg_index,1)
    angles_end  = control.IK(a)
    
    while (int(converter.convert_angle(angles_start[0]*(math.pi/180),leg_index)*(180/math.pi)) not in range(*sorted((int(min_),int(max_))))) or (int(converter.convert_angle(angles_end[0]*(math.pi/180),leg_index)*(180/math.pi)) not in range(*sorted((int(min_),int(max_))))) or angles_end[0] == 404 or angles_start[0] == 404:
        radiuses[i] = radiuses[i]+radius_increase
        
        end_point1 = np.array([radiuses[i]*math.sin(change+math.atan2(edge_point[0],edge_point[1])),radiuses[i]*math.cos(change+math.atan2(edge_point[0],edge_point[1])),walk_height])
        end_point2 = np.array([radiuses[i]*math.sin(-change+math.atan2(edge_point[0],edge_point[1])),radiuses[i]*math.cos(-change+math.atan2(edge_point[0],edge_point[1])),walk_height]) 
        a = converter.translate_coord(end_point1,leg_index,1)
        angles_start  = control.IK(a)
        a = converter.translate_coord(end_point2,leg_index,1)
        angles_end  = control.IK(a)
        idx = idx+1
        if idx > 10000:
            break    
            
    if Speed > 0:
        start_point = end_point2
        end_point = end_point1
    else:
        start_point = end_point1
        end_point = end_point2    
    
    points = start_point
    val = np.array([radiuses[i]*math.sin(Speed*(math.pi/180)+math.atan2(points[0],points[1])),radiuses[i]*math.cos(Speed*(math.pi/180)+math.atan2(points[0],points[1])),walk_height])
    points = np.vstack((points,val))
    
    for q in range(time-1):
        temp = points[q+1,:]
        val = np.array([radiuses[i]*math.sin(Speed*(math.pi/180)+math.atan2(temp[0],temp[1])),radiuses[i]*math.cos(Speed*(math.pi/180)+math.atan2(temp[0],temp[1])),walk_height])
        points = np.vstack((points,val))
        
    #===============Graph section ==============================

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
        # plt.plot((point_start[0]+e[m,0],e[m,0]),(point_start[1]+e[m,1],e[m,1]),'k-',lw=3)
        # plt.plot((point_end[0]+e[m,0],e[m,0]),(point_end[1]+e[m,1],e[m,1]),'k-',lw=3)
        # m = m+1
        
        
    # m= 0
    # for n in e:
        # if m == i:
            # plt.plot((point_mid[0]+e[m,0],e[m,0]),(point_mid[1]+e[m,1],e[m,1]),'b-',lw=2)
            # plt.plot(n[0],n[1],marker=".", markersize=20, markerfacecolor="blue", markeredgecolor = "blue")
        # else:
            # plt.plot(n[0],n[1],marker=".", markersize=20, markerfacecolor="black", markeredgecolor = "black")
        # m = m+1
        
    # plt.plot(point1[0],point1[1],marker="o", markersize=3, markerfacecolor="red",markeredgecolor = "red")
    # plt.plot(point2[0],point2[1],marker="o", markersize=3, markerfacecolor="red",markeredgecolor = "red")
    # plt.plot((point1[0],point2[0]),(point1[1],point2[1]),'r-',lw=2    )
    # plt.plot(edge_point[0],edge_point[1],marker="o", markersize=8, markerfacecolor="red",markeredgecolor = "red")
    

        
    # plt.plot(end_point[0],end_point[1],marker=".", markersize=8, markerfacecolor="cyan",markeredgecolor = "cyan")
    # plt.plot(start_point[0],start_point[1],marker="*", markersize=8, markerfacecolor="cyan",markeredgecolor = "cyan")    
    # for m in points:
       # plt.plot(m[0],m[1],marker=".", markersize=8, markerfacecolor="magenta",markeredgecolor = "magenta") 
    
    #plt.show()       
    
    
    return start_point,end_point,points
    
def rot2eul(R):
    beta = -np.arcsin(R[2,0])
    alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
    gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
    return np.array((alpha, beta, gamma))

cycle = 0 #goes up to 360

dist = (len_of_cycle-swing_time)*0.3
hoz_swing_vel = dist/swing_time
vert_swing_vel = hoz_swing_vel*HV_ratio

cycles = np.zeros([8,len_of_cycle])

for i in range(8):
    for j in range(len_of_cycle):
        if j+walking_starts[i] >= len_of_cycle:
            val = j+walking_starts[i] - len_of_cycle
        else:
            val = j+walking_starts[i]
        cycles[i][val] = j

#little bit o code to fix oversight and get angles working properly
walk_vel = abs(spider_speed)

if spider_speed < 0:
    angle = spider_angle
else:
    if spider_angle == 0:
        angle = math.pi
    else:
        angle = (math.pi-abs(spider_angle))*(-spider_angle/abs(spider_angle)) 

leg_dict = {
    0 : "leg_1R",
    1 : "leg_2R",
    2 : "leg_3R",
    3 : "leg_4R",
    4 : "leg_1L",
    5 : "leg_2L",
    6 : "leg_3L",
    7 : "leg_4L"

}

##======================================================================================


#fitness functiuons
def fitness_solo(agent):
    spider_speed = agent[0]#0

    #starting distance from the radius to search for valid turns
    mid_start_coord = agent[1]#50

    #amount of time leg is on ground when turning
    ground_turn_time = agent[2]#300

    #radius increase of serach when point is valid for rotation
    radius_increase = agent[3]#1

    #the walking height of the robot
    walk_height = agent[4]#-250

    #min searching radius for turn
    radius = agent[5]#200

    #ratio between width of ellipse path and height
    HV_ratio = agent[6]#1

    #time in the air of leg
    #swing_time = 50#40
    swing_time = agent[7]

    #start of each legs swing section
    #walking_starts = np.array([0,agent[7],0,agent[7],agent[7],0,agent[7],0])#np.array([0,45,90,135,180,225,270,315])
    #walking_starts = np.array([0,50,0,50,50,0,50,0])#np.array([0,45,90,135,180,225,270,315])
    walking_starts = np.array([agent[8],agent[9],agent[10],agent[11],agent[12],agent[13],agent[14],agent[15]])#np.array([0,45,90,135,180,225,270,315])


    #total units in cycle
    len_of_cycle = int(max(walking_starts)+swing_time+1)

    #setup init pose
    hips = [-36,-72,-108,-144,36,72,108,144]
    legs = [40,40]

    #intial search postions for leg
    set_pos = np.array([[agent[16],-agent[17],walk_height],#[-200,-100,walk_height]
                        [agent[18],-agent[19],walk_height],#[100,-200,walk_height]
                        [-agent[20],-agent[21],walk_height],#[-100,-200,walk_height]
                        [-agent[22],-agent[23],walk_height],#[-200,-100,walk_height]
                        [agent[24],agent[25],walk_height],#[200,100,walk_height]
                        [agent[26],agent[27],walk_height],#[100,200,walk_height]
                        [-agent[28],agent[29],walk_height],#[-100,200,walk_height]
                        [-agent[30],agent[31],walk_height]])#[-200,100,walk_height]
                    
    inital_looking = agent[32]
    
    cycle = 0 #goes up to 360
    
    dist = (len_of_cycle-swing_time)*0.3
    hoz_swing_vel = dist/swing_time
    vert_swing_vel = hoz_swing_vel*HV_ratio

    cycles = np.zeros([8,len_of_cycle])

    for i in range(8):
        for j in range(len_of_cycle):

            if j+walking_starts[i] >= len_of_cycle:
                val = j+walking_starts[i] - len_of_cycle
            else:
                val = j+walking_starts[i]
            try:
                cycles[i][int(val)] = j
            except:
                print(i)
                print(j)
                print(walking_starts[i])
                print(len_of_cycle)
                print(val)
                cycles[i][int(val)] = j

    #little bit o code to fix oversight and get angles working properly
    walk_vel = abs(spider_speed)

    if spider_speed < 0:
        angle = spider_angle
    else:
        if spider_angle == 0:
            angle = math.pi
        else:
            angle = (math.pi-abs(spider_angle))*(-spider_angle/abs(spider_angle))    
    
    i = 0
    leg_pos = np.array(set_pos,dtype='f')    
    count_log = 0
    curr_dist = 0
    time_log = 0
    robot.simulationReset()
    rotation_log = 0
    gyro_log = 0
    leg_torque_dif = np.zeros((1,8))
    body_torque_dif = np.zeros((8,8))
    while robot.step(timestep) != -1: 
        count_log = count_log + 1
        if robot.getTime() > fitness_time:
            out = 1000*(node.getPosition()[1]+8)-(node.getPosition()[0]*10)**2-rotation_log-gyro_log/2
            robot.simulationReset()
            break
        rot = node.getOrientation()
        rot2 = np.zeros((3,3))
        for t,v in enumerate(rot):
            rot2[math.floor(t/3),t - math.floor(t/3)*3] = v
        rotation_log = rotation_log + abs(math.pi/2-rot2eul(rot2)[2])
        gyro = control.get_gyro()
        gyro_log = gyro_log + abs(gyro[0]) + abs(gyro[1])
        for i in range(8):
            if turn== 0 :
                start_coord,end_coords = define_start_stop_coords(1,set_pos[i],dist,angle,edge_dict[i,0],edge_dict[i,1],i)
                #start_coord,end_coords = define_start_stop_coordsV3(1,dist,angle,i)
                leg_cycle = cycles[i][cycle]
                leg_name = leg_dict[i]
                # leg_angles = control.read(leg_name)
                # leg_touch = leg_angles[3]
                # leg_angles = leg_angles[0:3]
                
                speed = np.array([[walk_vel*math.cos(angle)],[walk_vel*math.sin(angle)]])
     
                if leg_cycle < swing_time:
                    time = swing_time - leg_cycle
                    a = leg_cycle
                    if a == 0:
                        a = 1
                    vel = (start_coord.reshape((3,1)) - leg_pos[i].reshape((3,1)))/time#time
                    speed = vel[0:2] 
            
                des_pos = leg_pos[i].reshape((3,1))+ np.matmul(np.eye(3,2),speed)
                if leg_cycle < swing_time:
                    curr_dist = (dist/swing_time)*leg_cycle
                    height = HV_ratio*(math.sqrt((dist/2)**2-(abs(dist/2-curr_dist))**2))
                    des_pos[2] = height+walk_height  
                    
                if leg_cycle == swing_time:
                    curr_dist = 0
                    des_pos = start_coord.reshape((3,1)) 

                des_pos_ = converter.translate_coord(des_pos,i,1)
        
                angles  = control.IK(des_pos_)
                if angles[1] == 404:
                    #print("invalid angles 1")
                    pass
                
                else:
                    angles = angles*(math.pi/180)
                    leg_pos[i] = des_pos.reshape((1,3))
                    leg = control.get_leg(leg_name)
                    leg.move(angles)
                    
                    
                
                
                
            else:
                leg_cycle = cycles[i][cycle]
                out = start_stop_turn(radius,turn_speed,ground_turn_time,i,walk_height)
                start = out[0]
                stop = out[1]
                turn_points = out[2]            
                leg_name = leg_dict[i]
                # leg_angles = control.read(leg_name)
                # leg_touch = leg_angles[3]
                # leg_angles = leg_angles[0:3]
                
                if leg_cycle < ground_turn_time:
                    des_pos = turn_points[int(leg_cycle)]
                    
                elif leg_cycle == 0:

                    des_pos = out[1].reshape((3,1))
                    
                else:

                    
                    delta = np.transpose(start) - np.transpose(leg_pos[i])
                    delta = delta/(len_of_cycle-leg_cycle)
                    
                    des_pos = leg_pos[i]+delta
                                    
                    dist = math.sqrt((start[0]-stop[0])**2+(start[1]-stop[1])**2)
                    curr_dist = (dist/(len_of_cycle-ground_turn_time))*(leg_cycle-ground_turn_time)
                    height = HV_ratio*(math.sqrt((dist/2)**2-(abs(dist/2-curr_dist))**2))
                    des_pos[2] = height+walk_height
                   
                    
                leg_pos[i] = des_pos
                des_pos_ = converter.translate_coord(des_pos,i,1)
        
                angles  = control.IK(des_pos_)
                angles = angles*(math.pi/180)
                leg = control.get_leg(leg_name)
                leg.move(angles)
                # print("des_pos") 
                # print(des_pos)
     
        cycle = cycle + 1
       
        if cycle >= len_of_cycle:
            cycle = 0

    #out = int(np.sum(agent)*100)
    return max(out,1),node.getPosition()[1]+8,node.getPosition()[0],rotation_log,gyro_log
    #1000*(node.getPosition()[1]+8)-(node.getPosition()[0]*10)**2-rotation_log-gyro_log/2


print("Data of each walk")
print("output = fitness,forward position,lateral position, rotation, gyro orientation")

print("--Basic")
out = fitness_solo(representation_basic)
print(out)

print("--Basic trained")
out = fitness_solo(representation_basic_trained)
print(out)

print("--Improved")
out = fitness_solo(representation_improved)
print(out)

print("--Improved trained")
out = fitness_solo(representation_improved_trained)
print(out)



