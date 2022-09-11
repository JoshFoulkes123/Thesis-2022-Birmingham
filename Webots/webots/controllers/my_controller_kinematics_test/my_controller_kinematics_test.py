"""steve_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot,Motor,PositionSensor
from math import pi,sin
import math
import numpy as np
from matplotlib import pyplot as plt

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

leg_Angles = np.array([[-0.698134],[-1.22173],[-1.91986],[-2.44346],[0.698],[1.22173],[1.91986],[2.44346]])

leg_Positions  = np.transpose(np.array([[115,-70,0],[55,-140,0],[-55,-140,0],[-115,-70,0],[115,70,0],[55,140,0],[-55,140,0],[-115,70,0]]))



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
        out = angle - (self.angles[index])
    
        return out
        
    def rotZ(self,ang):
        out = np.array([[math.cos(ang),-math.sin(ang),0],[math.sin(ang),math.cos(ang),0],[0,0,1]])
        return out
        


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
        
        self.positions = np.transpose(np.array([[-70,-115,0],[-140,-55,0],[-140,55,0],[-70,115,0],[70,-115,0],[140,-55,0],[140,55,0],[70,115,0]]))
        self.angles = np.array([[-2.26893],[-2.79253],[2.79253],[2.26893],[-0.872665],[-0.349066],[0.349066],[0.872665]])
        
        
        self.coord_converter = coord_conversion(self.angles,self.positions)

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
        
    def convert_coord(self,coords,index):
        
        out = self.coord_converter.translate_coord(coords,index)
        return out
    
    def move(self,leg_name,angles):
    
        #add position checker beofre moving

        
        if leg_name == "leg_1R":
            print(type(angles[0]))
            self.leg_1R.top.actuator.setPosition(float(angles[0]))
            self.leg_1R.mid.actuator.setPosition(float(angles[1]))
            self.leg_1R.bottom.actuator.setPosition(float(angles[2]))
        elif leg_name == "leg_2R":
            self.leg_2R.top.actuator.setPosition(float(angles[0]))
            self.leg_2R.mid.actuator.setPosition(float(angles[1]))
            self.leg_2R.bottom.actuator.setPosition(float(angles[2]))
        elif leg_name == "leg_3R":
            self.leg_3R.top.actuator.setPosition(float(angles[0]))
            self.leg_3R.mid.actuator.setPosition(float(angles[1]))
            self.leg_3R.bottom.actuator.setPosition(float(angles[2]))
        elif leg_name == "leg_4R":
            self.leg_4R.top.actuator.setPosition(float(angles[0]))
            self.leg_4R.mid.actuator.setPosition(float(angles[1]))
            self.leg_4R.bottom.actuator.setPosition(float(angles[2]))
        elif leg_name == "leg_1L":
            self.leg_1L.top.actuator.setPosition(float(angles[0]))
            self.leg_1L.mid.actuator.setPosition(float(angles[1]))
            self.leg_1L.bottom.actuator.setPosition(float(angles[2]))
        elif leg_name == "leg_2L":
            self.leg_2L.top.actuator.setPosition(float(angles[0]))
            self.leg_2L.mid.actuator.setPosition(float(angles[1]))
            self.leg_2L.bottom.actuator.setPosition(float(angles[2]))
        elif leg_name == "leg_3L":
            self.leg_3L.top.actuator.setPosition(float(angles[0]))
            self.leg_3L.mid.actuator.setPosition(float(angles[1]))
            self.leg_3L.bottom.actuator.setPosition(float(angles[2]))
        elif leg_name == "leg_4L":
            self.leg_4L.top.actuator.setPosition(float(angles[0]))
            self.leg_4L.mid.actuator.setPosition(float(angles[1]))
            self.leg_4L.bottom.actuator.setPosition(float(angles[2]))
          
          

            
    def read(self,leg_name):
    
        out = np.array([[0],[0],[0],[0]],dtype='f')
        if leg_name == "leg_1R":
            out[0]=self.leg_1R.top.position_sensor.getValue()
            out[1]=self.leg_1R.mid.position_sensor.getValue()
            out[2]=self.leg_1R.bottom.position_sensor.getValue()
            out[3]=self.leg_1R.touch_sensor.getValue()
        elif leg_name == "leg_2R":
            print("2")
            out[0]=self.leg_2R.top.position_sensor.getValue()
            out[1]=self.leg_2R.mid.position_sensor.getValue()
            out[2]=self.leg_2R.bottom.position_sensor.getValue()
            out[3]=self.leg_2R.touch_sensor.getValue()
        elif leg_name == "leg_3R":
            out[0]=self.leg_3R.top.position_sensor.getValue()
            out[1]=self.leg_3R.mid.position_sensor.getValue()
            out[2]=self.leg_3R.bottom.position_sensor.getValue()
            out[3]=self.leg_3R.touch_sensor.getValue()
        elif leg_name == "leg_4R":
            out[0]=self.leg_4R.top.position_sensor.getValue()
            out[1]=self.leg_4R.mid.position_sensor.getValue()
            out[2]=self.leg_4R.bottom.position_sensor.getValue()
            out[3]=self.leg_4R.touch_sensor.getValue()
        elif leg_name == "leg_1L":
            out[0]=self.leg_1L.top.position_sensor.getValue()
            out[1]=self.leg_1L.mid.position_sensor.getValue()
            out[2]=self.leg_1L.bottom.position_sensor.getValue()
            out[3]=self.leg_1L.touch_sensor.getValue()
        elif leg_name == "leg_2L":
            out[0]=self.leg_2L.top.position_sensor.getValue()
            out[1]=self.leg_2L.mid.position_sensor.getValue()
            out[2]=self.leg_2L.bottom.position_sensor.getValue()
            out[3]=self.leg_2L.touch_sensor.getValue()
        elif leg_name == "leg_3L":
            out[0]=self.leg_3L.top.position_sensor.getValue()
            out[1]=self.leg_3L.mid.position_sensor.getValue()
            out[2]=self.leg_3L.bottom.position_sensor.getValue()
            out[3]=self.leg_3L.touch_sensor.getValue()
        elif leg_name == "leg_4L":
            out[0]=self.leg_4L.top.position_sensor.getValue()
            out[1]=self.leg_4L.mid.position_sensor.getValue()
            out[2]=self.leg_4L.bottom.position_sensor.getValue()
            out[3]=self.leg_4L.touch_sensor.getValue()

        return out

converter = coord_conversion(leg_Angles,leg_Positions)
control = Leg_controller()  
  
def divergence(P1,P2,a):
    out  = abs(((P2[0]-P1[0])*(P1[1]-a[1])-(P1[1]-a[0])*(P2[1]-P1[1]))/(math.sqrt((P2[0]-P1[0])**2+(P2[1]-P1[1])**2)))
    return out

def define_start_stop_coords(want_stop,coord,distance,angle,start_edge_angle,end_edge_angle,leg_index):
    angle = angle+math.pi
    # a = converter.translate_coord(coord.reshape((3,1)),leg_index,1)
    # val = control.IK(a)

    start_edge_angle = start_edge_angle#as the robt runs off right hand but this runs off left hand
    end_edge_angle = end_edge_angle
    
    if start_edge_angle == 0:
        start_edge_angle = 180
    else:

        start_edge_angle = (180-abs(start_edge_angle))*(start_edge_angle/abs(start_edge_angle))
    

    if end_edge_angle == 0:
        end_edge_angle = 180
    else:
        end_edge_angle = (180-abs(end_edge_angle))*(end_edge_angle/abs(end_edge_angle))
    # coord[0] = -coord[0]
    # coord[1] = -coord[1]

    
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


    plt.plot(end_coord[0],end_coord[1],marker="x", markersize=20, markerfacecolor="blue")
    plt.plot(start_coord[0],start_coord[1],marker="x", markersize=20, markerfacecolor="red")
    plt.arrow(float(start_coord[0]),float(start_coord[1]),float(end_coord[0]-start_coord[0]),float(end_coord[1]-start_coord[1]),width = 0.05,length_includes_head = True)
    #check if valid
    ##check IK
    # a = converter.translate_coord(coord.reshape((3,1)),leg_index,1)
    # val = control.IK(a)

    
    point_start = np.array([math.sin(start_edge_angle*(math.pi/180))*100,math.cos(start_edge_angle*(math.pi/180))*100])
    point_end = np.array([math.sin(end_edge_angle*(math.pi/180))*100,math.cos(end_edge_angle*(math.pi/180))*100])
    
    a = converter.translate_coord(start_coord,leg_index,1)
    angles_start  = control.IK(a)
    a = converter.translate_coord(end_coord,leg_index,1)
    angles_end  = control.IK(a)

    min_ = min(start_edge_angle,end_edge_angle)
    max_ = max(start_edge_angle,end_edge_angle)+1

    #) and control.IK(end_coord)[0]==404 and control.IK(start_coord)[0]==404
    if (int(round(math.atan2(end_coord[0],end_coord[1]))*(180/math.pi)) not in range(*sorted((int(min_*(180/math.pi)),int(max_*(180/math.pi)))))) or angles_end[0] == 404 or angles_start[0] == 404:
        print("not valid")
        
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
        
        a = converter.translate_coord(start_coord,leg_index,1)
        angles_start  = control.IK(a)
        a = converter.translate_coord(end_coord,leg_index,1)
        angles_end  = control.IK(a)
        #and control.IK(end_coord)[0]==404 and control.IK(start_coord)[0]==404
        # print("-------------")
        # print(min_)
        # print((int(round(math.atan2(end_coord[0],end_coord[1])*(180/math.pi)))))
        # print(max_)
        # print(int(round(math.atan2(end_coord[0],end_coord[1])*(180/math.pi))) not in range(*sorted((int(min_),int(max_)))))
        # print(angles_end[0] == 404)
        # print(angles_start[0] == 404)
        # print(int(round(math.atan2(end_coord[0],end_coord[1])*(180/math.pi))) not in range(*sorted((int(min_),int(max_)))) or angles_end[0] == 404 or angles_start[0] == 404)
        # print(angles_start)
        # print(angles_end) 
        
        while(int(round(math.atan2(end_coord[0],end_coord[1])*(180/math.pi))) not in range(*sorted((int(min_),int(max_))))) or angles_end[0] == 404 or angles_start[0] == 404:#add 45 degree sheck
            # print("-------------")
            # print(min_)
            # print(math.atan2(end_coord[0],end_coord[1])*(180/math.pi))
            # print((int(round(math.atan2(end_coord[0],end_coord[1])*(180/math.pi)))))
            # print(max_)
            a = converter.translate_coord(start_coord,leg_index,1)
            angles_start  = control.IK(a)
            a = converter.translate_coord(end_coord,leg_index,1)
            angles_end  = control.IK(a)                  
            if idx > 1000:
                #move start value
                print("maximise")
                break
            end_coord = end_coord + selected
            start_coord = start_coord + selected
            idx = idx+1

        
    else:
        print("valid")

    ##check angles
    #if so return start coords


    
    
    plt.grid()
    plt.plot(end_coord[0],end_coord[1],marker="x", markersize=20, markerfacecolor="green")
    plt.plot(start_coord[0],start_coord[1],marker="x", markersize=20, markerfacecolor="yellow")
    plt.arrow(float(start_coord[0]),float(start_coord[1]),float(end_coord[0]-start_coord[0]),float(end_coord[1]-start_coord[1]),width = 0.05,length_includes_head = True)
    plt.plot((point_start[0],0),(point_start[1],0),'k-',lw=3)
    plt.plot((point_end[0],0),(point_end[1],0),'b-',lw=3)
    #plt.plot((point_mid[0],0),(point_mid[1],0),'b.',lw=3)
    
    #plt.show()
    
    return(start_coord.reshape((3,1)),end_coord.reshape((3,1)))
       
        



cycle = 0 #goes up to 360

#hyperparamters

walk_height = -250

angle = 0

walk_vel = -0.3

HV_ratio = 1

swing_time = 40

walking_starts = np.array([0,45,90,135,180,225,270,315])

dist = (360-swing_time)*0.3

hoz_swing_vel = dist/swing_time
vert_swing_vel = hoz_swing_vel*HV_ratio


cycles = np.zeros([8,360])

for i in range(8):
    for j in range(360):
        if j+walking_starts[i] >= 360:
            val = j+walking_starts[i] - 360
        else:
            val = j+walking_starts[i]
        cycles[i][val] = j

#setup init pose

hips = [-36,-72,-108,-144,36,72,108,144]
legs = [40,40]

dict = {
    0 : "leg_1R",
    1 : "leg_2R",
    2 : "leg_3R",
    3 : "leg_4R",
    4 : "leg_1L",
    5 : "leg_2L",
    6 : "leg_3L",
    7 : "leg_4L"

}

start_dict = {
    0 : -45,
    1 : -90,
    2 : -135,
    3 : -180,
    4 : 45,
    5 : 90,
    6 : 135,
    7 : 180

}

end_dict = {
    0 : 0,
    1 : -45,
    2 : -90,
    3 : -135,
    4 : 0,
    5 : 45,
    6 : 90,
    7 : 135,

}

set_pos = np.array([[-100,-200,walk_height],
                    [-200,-100,walk_height],
                    [-200,100,walk_height],
                    [-100,200,walk_height],
                    [100,200,walk_height],
                    [200,100,walk_height],
                    [200,-100,walk_height],
                    [100,-200,walk_height]])

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
print("Hello world")

# Main loop:
# - perform simulation steps until Webots is stopping the controller
idx = 0
while robot.step(timestep) != -1:
    idx = idx+1

    # test = np.array([[222.041+8.426],[-159.819-7.069],[-293.17089686]])
    # pos = converter.translate_coord(test.reshape((3,1)),0,1)
    # print(pos)
    
    # joints = control.read("leg_1R")
    # print(joints)
    
    # raw_pos = control.FK(joints[0],joints[1],joints[2])
    
    # print(raw_pos[0].reshape((3,1)))
    
    # pos = converter.translate_coord(raw_pos[0].reshape((3,1)),0,0)
    
    # print(pos)
    
    

    print("=====================")
    print(idx)
    leg_idx = 5
    #goal1 = np.array([[-55-math.sin(1.91986)*100],[140+math.cos(1.91986)*100],[-250]])
    #goal2 = np.array([[-55-math.sin(1.91986)*200],[140+math.cos(1.91986)*200],[-250]])
    goal1 = np.array([[150],[-150],[-250]])
    goal2 = np.array([[300],[-150],[-250]])
    goal1 = np.array([[50],[300],[-250]])
    goal2 = np.array([[200],[300],[-250]])
    update_goal = converter.translate_coord(goal1.reshape((3,1)),leg_idx,1)
    update_goal2 = converter.translate_coord(goal2.reshape((3,1)),leg_idx,1)
    #update_goal[0], update_goal[1] = update_goal[0], update_goal[1]
    print("update_goal")
    print(update_goal)
    print(update_goal2)
    print("check to original")
    print(converter.translate_coord(update_goal.reshape((3,1)),leg_idx,0))
    print(converter.translate_coord(update_goal2.reshape((3,1)),leg_idx,0))
    
    joints = control.IK(update_goal)
    joints2 = control.IK(update_goal2)
    print(joints)
    print(joints2)
    
    joints = joints*(math.pi/180)
    joints2 = joints2*(math.pi/180)
    print(joints)
    print(joints2)
    
    #control.leg_1R.top.actuator.setPosition(0.3)
    if idx== 1000:
        control.move("leg_2L",joints2)
        idx= 0 
    if idx > 500:
        goat = (idx-500)*((goal2-goal1)/500)+goal1
        update_goal = converter.translate_coord(goat,leg_idx,1)
        joints = control.IK(update_goal)
        joints = joints*(math.pi/180)
        control.move("leg_2L",joints)


        
    
    
    
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.
    
    # val = leg_sensors[1][2].getValues
    
    # print("val: "+val)
    # time.sleep(1)

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    
    
    
        
    #read cycle

    #get location of each leg using FK
    
    
    # for i in range(1):
    
        # start_coord,end_coords = define_start_stop_coords(1,set_pos[i],dist,angle,start_dict[i],end_dict[i],i)
    
        # leg_cycle = cycles[i][cycle]
        # leg_name = dict[i]
        # leg_angles = control.read(leg_name)
        # leg_touch = leg_angles[3]
        # leg_angles = leg_angles[0:3]
        # leg_pos = control.FK(leg_angles)
        # leg_pos = converter.translate_coord(leg_pos,i,0)
        
        
    # ##on return calculate speed depending upon position at the end of the walk cycle        
        # speed = np.array([[-walk_vel*math.sin(angle)],[walk_vel*math.cos(angle)]])
        # des_pos = leg_pos+ np.matmul(np.eye(3,2),speed)
        # if leg_cycle < swing_time:
            # time = swing_time - leg_cycle
            # vel = start_coord - leg_pos
            # speed = vel[0:2]
            
    # #apply speed
        
        
        # if leg_cycle < swing_time:
        
            # des_pos = start_coord
        
        
        
        



    # ##on return go to position a few units above and move down until senosor touch

    # ## on return check if foot touched bottom

    # ##if it has conntact it goes in forward direction until it unfeasable
    # ##the it returns and this point becomes start of swing
    # ##then calculate return swing speed to get the leg back to normal

    # #move using IK
    
        # des_pos = converter.translate_coord(des_pos,i,1)
        # try:
            # angles  = controller.IK(des_pos)
            # if angles[1] == 404:
                # #go to swing phase
                # #temporarily set all values from here to 0 to 0
                # print("invalid angles")
                # print(0/0)
            # else:
                # controller.move(leg_name,angles)
        
        # except:
            # print("invalid angles")
            # print(0/0)

    # cycle = cycle + 1
    # if cycle > 360:
        # cycle = 0
    # pass
    
    
# print("Done")    

# Enter here exit cleanup code.
