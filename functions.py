# Gregory Polmatier
import numpy as np

def trans(x,y):
    return np.array([[1,0,x],[0,1,y],[0,0,1]])

def rot(phi):
    return np.array([[np.cos(phi), -np.sin(phi), 0],
                    [np.sin(phi), np.cos(phi), 0],
                    [0,0,1]])
