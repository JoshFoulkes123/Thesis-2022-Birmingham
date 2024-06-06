import numpy as np
import math

Positions = np.array([[4,2,3,4],[-5,6,7,8],[0,0,0,0]])
print(Positions)

angles = np.array([2*math.pi,0.8,1.5,2.7])
print(angles)

class coord_conversion:
    def __init__(self,angs,pos):
        self.positions = pos
        self.angles = angs
        self.rotations = self.rotZ(angs[0])
        self.rotations = self.rotations[:,:,np.newaxis]
        self.angles = np.delete(angles,0)
        for ang in self.angles:
            val = self.rotZ(ang)
            val = val[:,:,np.newaxis]
            self.rotations =np.concatenate((self.rotations,val),2)
        
        self.angles= angs
        
    def translate_coord(self,coords,index):
        R = self.rotations[:,:,index]
        b = self.positions[:,index]
        out = np.matmul(-np.transpose(R),b.reshape(3,1))+(np.matmul(np.transpose(R),coords))
        return out
        
    def convert_angle(self,angle,index)
        out = angle - (self.angles[index])
    
        return out
        
    def rotZ(self,ang):
        out = np.array([[math.cos(ang),-math.sin(ang),0],[math.sin(ang),math.cos(ang),0],[0,0,1]])
        return out
        

        
tits = coord_conversion(angles,Positions)
coord = np.array([[10],[-10],[0]])
pepe = tits.translate_coord(coord,0)
print(pepe)

