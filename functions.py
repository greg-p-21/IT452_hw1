# Gregory Polmatier
import numpy as np
import math

class Point:
    def __init__(self, x, y):
        self.pt = np.matrix([[x],[y],[1]])

    def translate(self, m, n):
        z = np.eye(3)
        z[0,2] = m
        z[1,2] = n
        return np.dot(z, self.pt)

    def rotate(self, theta):
        z = np.matrix([[math.cos(theta), -math.sin(theta), 0], [math.sin(theta), math.cos(theta), 0], [0,0,1]])
        return np.dot(z, self.pt)

