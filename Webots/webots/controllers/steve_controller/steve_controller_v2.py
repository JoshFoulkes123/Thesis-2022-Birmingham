#import libraries
from controller import Robot,Motor,PositionSensor
import math
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import patches as pth

#forwards
spider_speed = 0.3
spider_angle = 0
#turn
turn = 1 #1 means turning
turn_speed = 0.07

#================================setup=======================

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


#Leg order: 1R,2R,3R,4R,1L,2L,3L,4L
#angles of each leg
leg_Angles = np.array([[-0.698134],[-1.22173],[-1.91986],[-2.44346],[0.698134],[1.22173],[1.91986],[2.44346]])
#xy position of each leg
leg_Positions  = np.array([[115,-70,0],[55,-140,0],[-55,-140,0],[-115,-70,0],[115,70,0],[55,140,0],[-55,140,0],[-115,70,0]])
#max and min of each leg
edge_dict = np.array([[-45,0],[-90,-45],[-135,-90],[-180,-135],[45,0],[90,45],[135,90],[180,135]])

#================================parameters to be trained==============================

mid_start_coord = 50




# ================================functions ====================================
#----------------coordinate converter---------------------
class coord_conversion:
    def __init__(self,angs,pos):
        self.positions = pos
        self.angles = angs
        self.rotations = self.rotZ(angs[0])
        self.rotations = self.rotations[:,:,np.newaxis]
        print()
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
        b = self.positions[:,index]

        if forward_back == 1:
            out = -np.matmul(np.transpose(R),b.reshape(3,1))+(np.matmul(np.transpose(R),coords.reshape(3,1)))
        else:
            out = (b.reshape(3,1)) + (np.matmul(R,coords.reshape(3,1)))
        # p = np.array([[0],[0],[0]])
        # p[0]= out[1]
        # p[1]=out[0]
        # p[2]=out[2]
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
    
    def FK(self,q0,q1,q2):
    
        DH_params = np.array([[55,-math.pi/2,0,q0],[130,0,0,q1],[200,0,0,q2]])
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
          
          
