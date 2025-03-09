import numpy as np

def interp(x1,x2,steps):
    out = np.zeros(steps)
    for i in range (steps):
        out[i] = x1 +(x2-x1)/(steps-1)*i
    return out
    
def interp1(x1,x2,i,steps):
    out = x1 +(x2-x1)/(steps-1)*i
    return out