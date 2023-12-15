from __future__ import print_function
import pyrealsense2 as rs
import numpy as np
import cv2
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import modern_robotics as mr
import time

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# Configure streams based on device capabilities
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Get camera intrinsics and depth scale
pipeline_profile = pipeline.get_active_profile()
intr = pipeline_profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
depth_sensor = pipeline_profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Set clipping distance for background removal based on depth
clipping_distance_in_meters = 1
clipping_distance = clipping_distance_in_meters / depth_scale

# Initialize robot manipulator
robot = InterbotixManipulatorXS("px100", "arm", "gripper")

# Move robot arm to a starting position, release gripper, and wait for 1 second
robot.arm.go_to_home_pose()
robot.gripper.release(2.0)
time.sleep(1)

# Define camera and robot coordinates for pen calibration
O_cx = 0.314575
O_cy = 0.324
O_cz = 0.2531

try:
    getting_pen = True
    while getting_pen:
        # Capture frames
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # Convert frames to numpy arrays
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Background removal based on depth
        depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), 153, color_image)

        # HSV thresholding
        hsv = cv2.cvtColor(bg_removed, cv2.COLOR_BGR2HSV)
        lower = np.array([110, 80, 8], np.uint8)
        upper = np.array([142, 180, 255], np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour
            areas = [cv2.contourArea(c) for c in contours]
            max_idx = np.argmax(areas)
            cnt = contours[max_idx]

            # Calculate centroid and moments of the contour
            M = cv2.moments(cnt)
            centroid_x = int(M['m10'] / M['m00'])
            centroid_y = int(M['m01'] / M['m00'])

            # Calculate depth of the centroid pixel
            pen_depth = aligned_depth_frame.get_distance(centroid_x, centroid_y)

            # Convert centroid pixel to 3D coordinates in camera frame
            pen_coords_wrt_camera = rs.rs2_deproject_pixel_to_point(intr, [centroid_x, centroid_y], pen_depth)
            P_cx, P_cy, P_cd = pen_coords_wrt_camera[0], pen_coords_wrt_camera[1], pen_coords_wrt_camera[2]

            # Convert camera coordinates to robot coordinates
            P_rx = O_cx - P_cx
            P_ry = O_cy - P_cd
            P_rz = O_cz - P_cy

            # Calculate robot's waist angle to align with the pen
            robot_current_angle = 0
            theta = np.arctan(P_ry / P_rx)
            angle_error = theta - robot_current_angle
            robot_current_angle = theta

            # Rotate robot's waist to face the pen
            robot.arm.set_single_joint_position('waist', robot_current_angle)

            # Get end effector position in robot coordinates
            joints = robot.arm.get_joint_commands()
            T = mr.FKinSpace(robot.arm.robot_des.M, robot.arm.robot_des.Slist, joints)
            [R_ee, p_ee] = mr.TransToRp(T)

            ee_x, ee_y, ee_z = p_ee[0], p_ee[1], p_ee[2]

            # Calculate displacement between end effector and pen location
            pe_error = [P_rx - ee_x, P_ry - ee_y, P_rz - ee_z]

            # Move the robot's end effector towards the pen
            robot.arm.set_ee_cartesian_trajectory(pe_error[0], 0, pe_error[2])

            margin = 0.02
            if (abs(pe_error[0]) <= margin) and (abs(pe_error[1]) <= margin) and (abs(pe_error[2]) <= margin):
                # Perform grasp action, move to sleep pose, release the gripper, and exit the loop
                getting_pen = False
                robot.gripper.grasp(2.0)
                robot.arm.go_to_sleep_pose()
                robot.gripper.release(2.0)

        # Display frames and contours
        cv2.imshow('Frame', color_image)
        cv2.imshow('Mask', mask)

        # Check for user key press to exit the loop
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            break

finally:
    cv2.destroyAllWindows()
    pipeline.stop()
