# Program that uses Forward kinematics to put the arm in a specified position
# and prin the results

# Cameron Berger and Greg Polmatier

import numpy as np

# return roll, pitch, yaw
# T: the matrix after final transformation
def findAngles(T):
    roll = 0 
    pitch = 0
    yaw = 0

    # pitch
    cosphi = np.sqrt(T[0,0]**2 + T[1,0]**2)
    pitch = np.arctan2(-T[2,0], cosphi)

    if pitch == np.pi/2:
        yaw = 0
        roll = np.arctan2(T[0,1], T[1,1])
    elif pitch == -np.pi/2:
        yaw = 0
        roll = -np.arctan2(T[0,1], T[1,1])
    else:
        # yaw
        yaw = np.arctan2(T[1,0]/cosphi, T[0,0]/cosphi)

        # roll
        roll = np.arctan2(T[2,1]/cosphi, T[2,2]/cosphi)

    return roll, pitch, yaw

# Create the final transformation matrix for the arm
# a1,a2, a3, a4: angle of the joints 1-4
# T: the matrix after the final transformation
def getMatrix(a1, a2, a3, a4):
    T = np.eye(4)

    return T

if __name__ == "__main__":
    a1 = input("Enter angle 1: ")
    a2 = input("Enter angle 2: ")
    a3 = input("Enter angle 3: ")
    a4 = input("Enter angle 4: ")

    # get final matrix
    m = getMatrix(a1, a2, a3, a4)
    x, y, Z = m[0,3], m[1,3], m[2,3]
    roll, pitch, yaw = findAngles(m)
    