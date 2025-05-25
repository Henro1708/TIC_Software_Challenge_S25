from TMMC_Wrapper import *
import rclpy
import numpy as np
import math
import time
from ultralytics import YOLO
# CONSTANTS ---------------------
DIST_SENSOR = 0.18
stopped_before = False
SIZE = 85
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

def turn_to_tag(tag_id):

    control.rotate(90.0, 1)

    # tags = camera.estimate_apriltag_pose(cv2_img)
    # if tags:
    #     for tag in tags:
    #         if tag[0] == tag_id:
    #             tag_id, range, bearing, elevation = tag
    #             print(f"Detected tag {tag_id} at range {range}, bearing {bearing}, elevation {elevation}")
    # while abs(bearing) > 2:
    #     tags = camera.estimate_apriltag_pose(cv2_img)
    #     if tags:
    #         for tag in tags:
    #             if tag[0] == tag_id:
    #                 tag_id, range, bearing, elevation = tag
    #                 print(f"Detected tag {tag_id} at range {range}, bearing {bearing}, elevation {elevation}")
    #     if bearing > 2:
    #         error = abs(bearing)
    #         control.send_cmd_vel(0.0, -0.05)
    #     elif bearing < -2:
    #         error = abs(bearing)
    #         control.send_cmd_vel(0.0, 0.05)
    #     print(f"bearing {bearing}")
    # while True:
    #     print("Stopping rotation")
    #     control.send_cmd_vel(0.0, 0.0)


try:
    if challengeLevel == 0:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Challenge 0 is pure keyboard control, you do not need to change this it is just for your own testing
                    

    if challengeLevel == 3:
        was_stopped = False
        currentTagIndex = 0
        #(tagid, turn angle, turn when dist to wall, tag aim offset)
        tagInfo = [(6,90,0.3,0.15), (7,90,0.3,0.15), (3,45,0.7,-0.15), (5,120,0.5,0.0)]
        control.send_cmd_vel(0.0, 0.0)
        def getTag():
            return tagInfo[currentTagIndex]

        def nextTag():
            global currentTagIndex, was_stopped
            currentTagIndex += 1
            was_stopped = False
            if currentTagIndex >= len(tagInfo):
                currentTagIndex = 0

        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 3 (or 3.5)
            # Get the latest lidar scan
            laserScan = lidar.checkScan()
            closestObs = lidar.detect_obstacle_in_cone(laserScan, 100, 0, 1)

            # Just exploration
            camera.checkImageRelease()
            cv2_img = camera.rosImg_to_cv2()
            is_stop_sign, x1, y1, x2, y2 = camera.ML_predict_stop_sign(cv2_img)
            tags = camera.estimate_apriltag_pose(cv2_img)
            if tags:
                for tag in tags:
                    if tag[0] == getTag()[0]:
                        tag_id, range, bearing, elevation = tag
                        print(f"Detected tag {tag_id} at range {range}, bearing {bearing}, elevation {elevation}")
            print(f"Range {range}")
            if was_stopped ==False and is_stop_sign and (x2-x1 > SIZE or y2-y1 > SIZE):
                control.send_cmd_vel(0.0, 0.0)
                was_stopped = True
                print("Stopping for traffic control")
                time.sleep(5.0)
            elif bearing > math.atan(getTag()[3]/range)*180/math.pi+1 and range>1.5:
                error = abs(bearing)
                control.send_cmd_vel(0.0, -0.05)
            elif bearing < math.atan(getTag()[3]/range)*180/math.pi-1 and range>1.5:
                error = abs(bearing)
                control.send_cmd_vel(0.0, 0.05)
            elif range>1.5:
                control.send_cmd_vel(0.5, 0.0)
            else:
                #lidar based control
                print("switching to lidar based control")
                laserScan = lidar.checkScan()
                closestObs = lidar.detect_obstacle_in_cone(laserScan, 100, 0, 1)
                print(f"Distance to front is {closestObs[0]}")
                if closestObs[0] > getTag()[2]:
                    control.send_cmd_vel(0.1, 0.0)
                else:
                    control.send_cmd_vel(0.0, 0.0)
                    print(f"Stopping at tag {getTag()[0]}")
                    control.rotate(getTag()[1], 1)
                    nextTag()
                    print(f"Next tag is {getTag()[0]}")




except KeyboardInterrupt:
    print("Keyboard interrupt received. Stopping...")

finally:
    control.stop_keyboard_control()
    robot.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()