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
   trans_mat = np.matrix([[1, 0, 0, p_x],
                          [0, 1, 0, p_y],
                          [0, 0, 1, p_z],
                          [0, 0, 0, 1]])
   return trans_mat


def makeT(theta, l, d, alpha):
   r_z = rotZ(theta)
   tr  = translate(l, 0, d)
   r_x = rotX(alpha)
   return np.matmul(np.matmul(r_z, tr), r_x)

def makeBigT(th_1, th_2, th_3, th_4):
   rot_z = math.atan(0.128/0.024)
   
   return np.matmul(np.matmul(np.matmul(T0, T1), T2), T3)
