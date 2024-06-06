import np as numpy
import math


def FK(q0,q1,q2):
    
    DH_params = np.array([[55,-math.pi/2,0,q0],[130,0,0,q1],[200,0,0,q2]])
    out = np.identity(4)

    for i in range(3):
        print(i)
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

print(FK(0,0.8,0.8))  