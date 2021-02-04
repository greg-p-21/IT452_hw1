# Program that uses Forward kinematics to put the arm in a specified position
# and prin the results

# Cameron Berger and Greg Polmatier

import numpy as np
import refframe

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


if __name__ == "__main__":
    a1 = float(input("Enter angle 1: "))
    a2 = float(input("Enter angle 2: "))
    a3 = float(input("Enter angle 3: "))
    a4 = float(input("Enter angle 4: "))

    # get final matrix
    m = refframe.makeBigT(a1, a2, a3, a4)
    x, y, Z = m[0,3], m[1,3], m[2,3]
    roll, pitch, yaw = findAngles(m)
    output = 'position (' + repr(x) + ', ' + repr(y) + ', ' + repr(z) + ')'
    output += 'orientation (' + repr(roll) + ', ' + repr(pitch) + ', ' + repr(yaw) + ')'
    print(output) 