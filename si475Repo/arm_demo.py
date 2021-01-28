from arm_controller import ArmController

ac=ArmController()
ac.set_joints([0,0,0,0])
print(ac.get_pose())

