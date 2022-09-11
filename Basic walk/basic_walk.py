import numpy as np
import math

cycle = 0 #goes up to 360

#hyperparamters

angle = 0

walk_vel = 0.3

HV_ratio = 1.1

swing_time = 40

walking_starts = np.array([0,45,90,135,180,225,270,315])

dist = (360-swing_time)*0.3

hoz_swing_vel = dist/swing_time
vert_swing_vel = hoz_swing_vel*HV_ratio

control = Leg_controller()

def define_start_stop_coords(want_start_or_stop,coord,distance,angle,start_edge_angle,end_edge_angel,leg_index):

    #the coordinates is in relation to the origin of the robot and in the robots coordinate system
    #1 means start is defined and looking for stop
    a = coord_converter.translate_coord(coord,leg_index)
    val = IK(a)
    if val[0] == 404:
        print("original input coordinate not valid")
        return(404)
    
    
    if want_start_or_stop == 1:
        edge_angle = start_edge_angle
        move_angle= angle
        end_coord = np.array([[0],[0],[0]])
    else:
        edge_angle = end_edge_angel
        move_angle = 180+angle
        end_coord = coord
    
    other_coord = np.array([[0],[0],[0]])
    other_coord[2] = int(coord[2])
    x_change = math.sin(edge_angle)
    y_change = math.cos(edge_angle)

    other_coord[0]= math.sin(move_angle)*distance
    other_coord[1]= math.cos(move_angle)*distance
    
    a = coord_converter.translate_coord(other_coord,leg_index)
    val = IK(a)
    
    if val[0] != 404:
        return(coord,other_coord)
        break
    else: 
        while val[0] == 404:
            other_coord[0] = other_coord[0]+x_change
            other_coord[1] = other_coord[1]+y_change
            val = IK(other_coord)
        return(coord,other_coord)
            
    
    #create start and end points
    #check if valid
    #if not nove point one unit in direction until both valid
    
    #return points


#setup cycle graphhs

cycles = np.zeros([8,36])

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


#main loop
while(1):
    
#read cycle

#get location of each leg using FK
    for i in range(8):
        a = control.read.
        

##on return calculate speed depending upon position at the end of the walk cycle


#apply speed

##on return go to position a few units above and move down until senosor touch

## on return check if foot touched bottom

##if it has conntact it goes in forward direction until it unfeasable
##the it returns and this point becomes start of swing
##then calculate return swing speed to get the leg back to normal

#move using IK

    cycle = cycle + 1
    if cycle > 360:
        cycle = 0


    





