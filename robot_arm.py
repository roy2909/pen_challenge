from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_modules.xs_robot.gripper import InterbotixGripperXS
import time
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import modern_robotics as mr
# The robot object is what you use to control the robot
robot = InterbotixManipulatorXS("px100", "arm", "gripper")
mode = 'h'
# Let the user select the position
while mode != 'q':
    mode=input("[h]ome, [s]leep, [q]uit, [c]lose,[o]pen")
    if mode == "h":
        robot.arm.go_to_home_pose()
    elif mode == "s":
        robot.arm.go_to_sleep_pose()
    elif mode =='o':
        robot.gripper.release()
    elif mode=='c':
        robot.gripper.grasp()
    elif mode=='d':
        robot.gripper.grasp(2.0)
        time.sleep(8)
        robot.gripper.release(2.0)
        joints = robot.arm.get_joint_commands()
        T = mr.FKinSpace(robot.arm.robot_des.M, robot.arm.robot_des.Slist, joints)
        [R, p] = mr.TransToRp(T) # get the rotation matrix and the displacement
        print(f'pen coords wrt robot base: {p}')
# # pen coords wrt robot base
        P_rx, P_ry, P_rz = p[0], p[1], p[2]
        print(P_rx, P_ry, P_rz)



        

    
 
