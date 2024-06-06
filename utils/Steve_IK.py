import math 

def IK(pos):
    try:
        x = pos[0]
        y = pos[2]
        z = pos[1]
        
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
        q1 = -(math.atan(x/y)*(180/math.pi))
    except:
        q3=q2=q1 =404
    
    out = np.array([[round(q1,1)],[round(q2,1)],[round(q3,1)]])
    
    return out
    
print(IK(100,110,-110))