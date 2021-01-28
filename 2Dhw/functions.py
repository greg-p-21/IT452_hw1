# Gregory Polmatier
import numpy as np

def trans(x,y):
    return np.array([[1,0,x],[0,1,y],[0,0,1]])

def rot(phi):
    return np.array([[np.cos(phi), -np.sin(phi), 0],
                    [np.sin(phi), np.cos(phi), 0],
                    [0,0,1]])

def __main__():
    origin = np.array([[0],[0],[1]])
    phi1 = np.pi/4  
    phi2 = -np.pi/6
    print(np.eye(3).dot(rot(phi1)).dot(trans(5,0)).dot(rot(phi2)).dot(trans(3,0)))
