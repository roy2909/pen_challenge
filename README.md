## Automated Grasping: Pincher X100 4-DOF Robot Arm -Picks a Purple Pen
Author: Rahul Roy

Programmed a Pincher X100 4-DOF robot arm to grasp a purple colored pen.

# Quickstart
1. Clone the project into your workspace.
2. To start the arm, open a new terminal window and run `ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100`.
You should see an rviz window pop up. This window shows the location of the arm.
To Stop running the arm, press C-c in the terminal window.
3. Calibrate `O_cx`, `O_cy`, `0_cz` which are coordinates of the base of the robot with respect to the base of the camera in the x, y, z direction (Measure the distance in each respective direction)
4. Make sure you are in the same directory as the cloned workspace and run python3 `pen_detection_with_filter.py`.
5. Shown below is the Arm identifying and grasping the purple pen.

https://github.com/roy2909/pen_challenge/assets/144197977/a6163c0b-eacd-40e8-9625-2173ba70d403