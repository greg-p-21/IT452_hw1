# Program that uses Inverse kinematics to find joint angles
# and print the results

# Grace Hove and Greg Polmatier

import numpy as np
from arm_controller import ArmController

def makeT(x, y, z, psi, phi, theta):
    cpsi    = np.cos(psi)
    cphi    = np.cos(phi)
    ctheta  = np.cos(theta)
<<<<<<< HEAD
    
=======

>>>>>>> 5f59a84108885d5475ceb555288a941e3b78cae8
    spsi    = np.sin(psi)
    sphi    = np.sin(phi)
    stheta  = np.sin(theta)

    # print("angles", psi, phi, theta)

    return np.matrix([  [ctheta*cphi, -stheta*cpsi + ctheta*sphi*spsi,  stheta*spsi + ctheta*sphi*cpsi,  x],
                        [stheta*cphi,  ctheta*cpsi + stheta*sphi*spsi, -ctheta*spsi + stheta*sphi*cpsi,  y],
                        [-sphi,        cphi*spsi,                       cphi*cpsi,                       z],
                        [0,             0,                              0,                               1]])

def getAngles(T):
    # lengths 
    D1 = 0.077
    L2 = 0.195673197
    L3 = 0.126

    # angle 1
    a1 = np.arctan2(-T[0,1], T[1,1])
    
    # angle 2
<<<<<<< HEAD
    s2 = (T[2,3] - D1 - T[2,0] * L3) / L2	
    # s2 = -1*(L3*T[0,2] + D1 - T[0,3]) / L2				
=======
    s2 = (T[2,3] - D1 - T[2,0] * L3) / L2   
    # s2 = -1*(L3*T[0,2] + D1 - T[0,3]) / L2                
>>>>>>> 5f59a84108885d5475ceb555288a941e3b78cae8
    c2 = ((T[0,3]/T[1,1]) - (T[2,2] * L3)) / L2
    # c2 = (T[0,3] - np.cos(a1)*L3*T[2,2])/ L2
    a2 = np.arctan2(s2, c2)

    # angle 3
    a3 = np.arctan2(T[2,0], T[2,2]) - a2 

    return a1, a2, a3

if __name__ == "__main__":
    x = float(input("Enter x: "))
    y = float(input("Enter y: "))
    z = float(input("Enter z: "))

    roll = float(input("Enter roll: "))
    pitch = float(input("Enter pitch: "))
    yaw = float(input("Enter yaw: "))

    T = makeT(x, y, z, roll, pitch, yaw)

    # print(T)

    theta1, theta2, theta3 = getAngles(T)

    # print(theta1, theta2, theta3)

<<<<<<< HEAD
    a1 = abs(theta1 + 0)
    a2 = abs(theta2 - .7131)
    a3 = abs(theta3 + .7131)

    angles = [a1,a2,0,a3]
    print(angles)


    ac = ArmController()
    ac.set_joints(angles)

    # output = '\n(' + repr(a1) + ', ' + repr(a2) + ', ' + repr(a3) + ')'


=======
    a1 = theta1 + 0
    a2 = theta2 - .7131
    a3 = theta3 + .7131

    angles = [a1,a2,0,a3]
    print(angles)
    ac = ArmController()
    ac.set_joints(angles)
    print(ac.get_pose())
>>>>>>> 5f59a84108885d5475ceb555288a941e3b78cae8
