from TMMC_Wrapper import *
import rclpy
import numpy as np
import math
import time
from ultralytics import YOLO
# CONSTANTS ---------------------
DIST_SENSOR = 0.18
stopped_before = False
SIZE = 100
# Variable for controlling which level of the challenge to test -- set to 0 for pure keyboard control
challengeLevel = 3

# Set to True if you want to run the simulation, False if you want to run on the real robot
# is_SIM = False

# Set to True if you want to run in debug mode with extra print statements, False otherwise
# Debug = True

# Initialization    
if not "robot" in globals():
    robot = Robot(IS_SIM=True, DEBUG=False)
    
control = Control(robot)
camera = Camera(robot)
imu = IMU(robot)
logging = Logging(robot)
lidar = Lidar(robot)


def stage_one_wall_avoidanc():
    print("Wall avoided")

    while rclpy.ok(): 
        rclpy.spin_once(robot, timeout_sec=0.1) 
        time.sleep(0.1) 
    #Write your solution here for challenge level # 1 
    # It is recommended you use functions for aspects of the challenge that will be resused in later c 
    # # For example, create funct Hetoo close to a wall ○ X 
    laserScan = lidar.checkScan() 
    closestobs = lidar.detect_obstacle_in_cone(laserScan,1.0,0,20) 
    # print(closestobs) 
    if (closestobs[0]<0.4 and closestobs[0]>0.0): 
        control.stop_keyboard_control() 
        # control.move_backward() 
        control.set_cmd_vel( 0.1, 0, 0.5) 
        control.set_cmd_vel( -0.5, 0, 1) 
        time. sleep (1)
        control.start_keyboard_input()
                                                                             

if challengeLevel <= 2:
    control.start_keyboard_control()
    rclpy.spin_once(robot, timeout_sec=0.1)


try:
    if challengeLevel == 0:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Challenge 0 is pure keyboard control, you do not need to change this it is just for your own testing

    if challengeLevel == 1:
        THRESHOLD = 0.5  # Distance threshold in meters
        retreating = False  # State variable
        control.start_keyboard_input()
        
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)

            # Get the latest lidar scan
            laserScan = lidar.checkScan()
            
            # Detect the closest obstacle in a cone in front (center=0, ±30 degrees)
            closestObs = lidar.detect_obstacle_in_cone(laserScan, 100, 0, 30)
            
            # If an obstacle is detected and too close
            if closestObs[0] > 0.0 and closestObs[0] < THRESHOLD:
                if not retreating:
                    print("Obstacle too close, retreating")
                    control.stop_keyboard_input()  # Disable keyboard input
                    retreating = True
                control.send_cmd_vel(-0.3, 0.0)  # Move backward
            else:
                if retreating:
                    print("Safe, resuming keyboard control")
                    control.send_cmd_vel(0.0, 0.0)  # Stop the robot
                    control.start_keyboard_input()  # Re-enable keyboard input
                    retreating = False

                

    if challengeLevel == 2:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            # time.sleep(0.1)
            # Write your solution here for challenge level 2
            cv2_img = camera.rosImg_to_cv2()
            is_stop_sign, x1, y1, x2, y2 = camera.ML_predict_stop_sign(cv2_img)
            stop_near = False   # This is used to check if the stop sign is close enough
            if (x2-x1 > SIZE and y2-y1 > SIZE):
                stop_near = True
                
            if stop_near:
                if not stopped_before:   # This is used to check if we stoped for this stop sign in the past
                    stopped_before = True
                    control.set_cmd_vel(0,0,5)
            else:
                stopped_before = False
                    

    if challengeLevel == 3:
        FIRST_TAG = 6
        SECOND_TAG = 7
        FOURTH_TAG = 5
        known_tags = [FIRST_TAG, SECOND_TAG, FOURTH_TAG]

        most_recent_tag = None
        did_i_stop = False
        currently_tag_rotating = False

        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 3 (or 3.5)
            # Get the latest lidar scan
<<<<<<< HEAD
            laserScan = lidar.checkScan()
            
            # Detect the closest obstacle in a cone in front (center=0, ±30 degrees)
            closestObs = lidar.detect_obstacle_in_cone(laserScan, 100, 0, 30)
            # print(closestObs[0])
            

            # Just exploration
            current_img = camera.rosImg_to_cv2()

            tag_dict = {6: 90, 7: 90, 5: 135}

            tags = camera.estimate_apriltag_pose(current_img)
            if tags is not None:
                for tag in tags:
                    if tag[1] > 0.3:
                        control.send_cmd_vel(0.2, 0.0)
                        print(f"too far away, distance is {tag[1]}")
                    else:
                        control.rotate(tag_dict[tag[0]], 1)
                        print("rotating!")
            else:
                if closestObs[0] > 0.5:
                    control.send_cmd_vel(0.2, 0.0)
                else:
                    print("Obstacle too close, retreating")
                    control.send_cmd_vel(0.0, 0.0)
                    control.rotate(5, 1)

=======
            cv2_img = camera.rosImg_to_cv2()
            is_stop_sign, x1, y1, x2, y2 = camera.ML_predict_stop_sign(cv2_img)
>>>>>>> fe16b7a (day 2)

            laserScan = lidar.checkScan()
            closestObs = lidar.detect_obstacle_in_cone(laserScan, 100, 0, 30)
            # print(closestObs[0])

<<<<<<< HEAD
            # i0.0 < f closestObs[0] < 0.4
=======

            # Just exploration
            current_img = camera.rosImg_to_cv2()
            tag_dict = {FIRST_TAG: 90, SECOND_TAG: 90, FOURTH_TAG: 135}

            tags = camera.estimate_apriltag_pose(current_img)

            if tags:
                print(f'I FOUND A TAG! It is tag #{tags[0]}')
                for tag in tags:
                    most_recent_tag = tag[0]

            if is_stop_sign and (x2-x1 > SIZE and y2-y1 > SIZE) and not did_i_stop:
                control.send_cmd_vel(0.0, 0.0)  # Stop the robot
                did_i_stop = True
                time.sleep(3.0)
            else:
                if closestObs[0] > 0.6:
                    control.send_cmd_vel(0.2, 0.0)

                else:
                    control.send_cmd_vel(0.0, 0.0)
                    if most_recent_tag is not None:
                        if most_recent_tag in known_tags:
                            print("I am rotating because I saw the tag.")
                            control.rotate(tag_dict[most_recent_tag], 1)
                            time.sleep(3)
                        else:
                            print("TAG NOT SEEN")
                            #control.rotate(90, 1)
                    else:
                        print("I am rotating because im simply too close to the wall. ")
                        control.rotate(5, 1)
>>>>>>> fe16b7a (day 2)


    if challengeLevel == 4:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 4



    if challengeLevel == 5:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 5
            

except KeyboardInterrupt:
    print("Keyboard interrupt received. Stopping...")

finally:
    control.stop_keyboard_control()
    robot.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()