
from __future__ import print_function
import pyrealsense2 as rs
import numpy as np
import cv2
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import modern_robotics as mr
import time

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)


# Start streaming
cfg = pipeline.start(config)

profile = cfg.get_stream(rs.stream.color)
intr = profile.as_video_stream_profile().get_intrinsics()

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = cfg.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

# The robot object is what you use to control the robot
robot = InterbotixManipulatorXS("px100", "arm", "gripper")

# move arm to starting position
robot.arm.go_to_home_pose()
robot.gripper.release(2.0)
time.sleep(1)


# Calibrate the pen and robot wrt to the camera to get the base coordinates of camera x,y,z


O_cx = 0.314575
O_cy = 0.324
O_cz = 0.2531


## Streaming loop

try:
    getting_pen = True
    while getting_pen:
    
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # depth frame to get pixel distance later
        dpt_frame = aligned_depth_frame.as_depth_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        # Render images:
        #   depth align to color on left
        #   depth on right
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))

        # Thresholding
        hsv = cv2.cvtColor(images, cv2.COLOR_BGR2HSV)

        lower = np.array([110, 80, 8], np.uint8)
        upper = np.array([142, 180, 255], np.uint8)

        mask = cv2.inRange(hsv, lower, upper)
        res = cv2.bitwise_and(images, images, mask = mask)

        # Contours
        ret, thresh = cv2.threshold(mask, 127, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            areas = [cv2.contourArea(c) for c in contours]
            max_idx = np.argmax(areas)
            cnt = contours[max_idx]
            cv2.drawContours(images, [cnt], 0, (0,255,0), 3)

            M = cv2.moments(cnt)
            try:
                # get centroid coords and display it
                centroid_x = int(M['m10']/M['m00'])
                centroid_y = int(M['m01']/M['m00'])
                cv2.circle(images, (centroid_x, centroid_y), 10, (255,0,0), -1)
                # print(f'centroid found: {centroid_x}, {centroid_y}') 

                # get depth of centroid in pixel coordinates
                # pixel_distance_in_meters
                pen_depth = dpt_frame.get_distance(centroid_x, centroid_y)

                # get location of centroid wrt camera in cartesian coordinates
                pen_coords_wrt_camera = rs.rs2_deproject_pixel_to_point(intr, [centroid_x, centroid_y], pen_depth)


                # pen coords wrt camera
                P_cx, P_cy, P_cd = pen_coords_wrt_camera[0], pen_coords_wrt_camera[1], pen_coords_wrt_camera[2]

                # pen coords wrt robot
                P_rx = O_cx - P_cx
                P_ry = O_cy - P_cd
                P_rz = O_cz - P_cy

                robot_current_angle = 0
                # theta = waist rotation in radians
                theta = np.arctan(P_ry/P_rx)
                print(f'theta: {theta}')
                angle_error = theta - robot_current_angle
                robot_current_angle = theta

                # turn at waist until end-effector is facing the pen
                robot.arm.set_single_joint_position('waist', robot_current_angle)

                # get end effector position
                joints = robot.arm.get_joint_commands()
                T = mr.FKinSpace(robot.arm.robot_des.M, robot.arm.robot_des.Slist, joints)
                [R_ee, p_ee] = mr.TransToRp(T) # get the rotation matrix and the displacement
                # print(f'end effector coords wrt robot base: {p_ee}')

                ee_x, ee_y, ee_z = p_ee[0], p_ee[1], p_ee[2]
                # displacement between end effector and pen location
                pe_error = [P_rx - ee_x, P_ry - ee_y, P_rz - ee_z]

                # move toward pen until pen is inside grippers
                robot.arm.set_ee_cartesian_trajectory(pe_error[0], 0, pe_error[2])

                margin = 0.02
                if (pe_error[0] <= margin) and (pe_error[1] <= margin) and (pe_error[2] <= margin):
                    getting_pen = False
                    print('next to pen')
                    robot.gripper.grasp(2.0)
                    robot.arm.go_to_sleep_pose()
                    robot.gripper.release(2.0)
                    break
              
            except:
                pass


            # Display
            cv2.imshow('frame', images)
            # cv2.imshow('mask', mask)
            # cv2.imshow('res', res)
        
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

finally:
    pipeline.stop()