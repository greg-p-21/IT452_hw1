import numpy as np
import math

def rotX(angle):
   c = math.cos(angle)
   s = math.sin(angle)
   rot_mat = np.matrix([[1, 0, 0,  0],
                        [0, c,-1*s,0],
                        [0, s, c,  0],
                        [0, 0, 0,  1]])
   return rot_mat

def rotZ(angle):
   c = math.cos(angle)
   s = math.sin(angle)
   rot_mat = np.matrix([[c,-1*s,0, 0],
                        [s,  c, 0, 0],
                        [0,  0, 1, 0],
                        [0,  0, 0, 1]])
   return rot_mat

def translate(p_x, p_y, p_z):
   trans_mat = np.matrix([[1,  0, 0, p_x],
                           [0,  1, 0, p_y],
                           [0,  0, 1, p_z],
                           [0,  0, 0, 1]])
   return trans_mat


def makeT(theta, l, d, alpha):
   r_z = rotZ(theta)
   tr  = translate(l, 0, d)
   r_x = rotX(alpha)
   return np.matmul(r_z, tr, r_x)

def makeBigT(th_r, th_1, th_2, th_3):
    T_r = makeT(th_r, 0, 0, 0)
    T_1 = makeT(th_1, 0, 0.077, -1*(math.pi/2))
    T_2 = makeT(th_2+1.385, 1.3, 0, 0)
    T_3 = makeT(th_3-1.385, 0.124, 0, 0)
    T_4 = makeT(0, 0.126, 0, 0)
    return np.matmul(T_r, T_1, T_2, T_3, T_4)
