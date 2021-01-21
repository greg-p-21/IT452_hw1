from arm_controller import ArmController

ac=ArmController()
ac.set_joints([.2,-.5,.3,-.4])
print(ac.get_pose())
ac.set_joints([0,0,0,0])
print(ac.get_pose())
ac.set_joints([-.2,.5,-.3,.4])
print(ac.get_pose())
ac.open_tool()
print(ac.get_pose())
