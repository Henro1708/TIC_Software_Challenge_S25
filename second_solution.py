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
challengeLevel = 2

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
    print(closestobs) 
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
                control.send_cmd_vel(linear_x=-0.3, angular_z=0.0)  # Move backward
            else:
                if retreating:
                    print("Safe, resuming keyboard control")
                    control.send_cmd_vel(linear_x=0.0, angular_z=0.0)  # Stop the robot
                    control.start_keyboard_input()  # Re-enable keyboard input
                    retreating = False

                

    if challengeLevel == 2:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
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
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 3 (or 3.5)

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