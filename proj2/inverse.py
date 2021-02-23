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
    m_2 = refframe.makeBiggererT(0,0,0,0,0)
    final_pos = np.matmul(m, np.array([0,0,0,1]).transpose())
    final_pos_2 = np.matmul(m_2, np.array([0,0,0,1]).transpose())
    roll, pitch, yaw = findAngles(m)
    output = '\nposition (' + repr(final_pos[0,0]) + ', ' + repr(final_pos[0,1]) + ', ' + repr(final_pos[0,2]) + ')\n'
    output += 'orientation (' + repr(roll) + ', ' + repr(pitch) + ', ' + repr(yaw) + ')\n'
    #output += '\nposition_2 (' + repr(final_pos_2[0,0]) + ', ' + repr(final_pos_2[0,1]) + ', ' + repr(final_pos_2[0,2]) + ')\n'
    print(output)
