from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import time
import modern_robotics as mr

# Initialize the robot object for controlling the arm and gripper
robot = InterbotixManipulatorXS("px100", "arm", "gripper")

def control_robot():
    """
    Control the robot's arm and gripper based on user input.
    'h': Go to home pose
    's': Go to sleep pose
    'o': Open gripper
    'c': Close gripper
    'd': Demonstrate a grasp-release sequence
    'q': Quit the program
    """
    mode = ''
    while mode != 'q':
        mode = input("[h]ome, [s]leep, [q]uit, [c]lose, [o]pen, [d]emonstrate: ")
        if mode == "h":
            robot.arm.go_to_home_pose()
        elif mode == "s":
            robot.arm.go_to_sleep_pose()
        elif mode == 'o':
            robot.gripper.release()
        elif mode == 'c':
            robot.gripper.grasp()
        elif mode == 'd':
            # Demonstrate a grasp-release sequence
            robot.gripper.grasp(2.0)
            time.sleep(8)
            robot.gripper.release(2.0)
            # Get end-effector position in robot coordinates
            joints = robot.arm.get_joint_commands()
            T = mr.FKinSpace(robot.arm.robot_des.M, robot.arm.robot_des.Slist, joints)
            [R, p] = mr.TransToRp(T)
            print(f'Pen coordinates wrt robot base: {p}')
            P_rx, P_ry, P_rz = p[0], p[1], p[2]
            print(P_rx, P_ry, P_rz)

if __name__ == "__main__":
    control_robot()
