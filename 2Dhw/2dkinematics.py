# Gregory Polmatier
# 2D Kinematics Homework

import numpy as np

def trans(x,y):
    return np.array([[1,0,x],[0,1,y],[0,0,1]])

def rot(phi):
    return np.array([[np.cos(phi), -np.sin(phi), 0],
                    [np.sin(phi), np.cos(phi), 0],
                    [0,0,1]])

if __name__ == "__main__":
    numJoints = input("How many joints? ")

    angles = []
    lengths = []
    for i in range(int(numJoints)):
        angles.append(float(input("Angle in radians? ")))
        lengths.append(float(input("Length of link? ")))

    endMatrix = np.eye(3)
    for a, l in zip(angles, lengths):
        endMatrix = endMatrix.dot(rot(a)).dot(trans(l,0))

    endMatrix.dot(np.array([[0],[0],[1]]))
    print("The end of the arm is at (", endMatrix[0,2], ",", endMatrix[1,2], ")")

    print("The angle from the X-axis is", np.arctan2(endMatrix[1,0], endMatrix[0,0]), 
                                                                        "radians")
