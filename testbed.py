if challengeLevel == 1:
    
    while rclpy.ok():
        rclpy.spin_once(robot, timeout_sec=0.1)
        time.sleep(0.1)

        # Write your solution here for challenge level 1
        # It is recommended you use functions for aspects of the challenge that will be resused in later challenges
        # For example, create a function that will detect if the robot is too close to a wall
        laserScan = lidar.checkScan()
        control.start_keyboard_input()
        obj_dist = lidar.detect_obstacle_in_cone(laserScan,1.0,0,20)[0]
        print(obj_dist)

        while (obj_dist > 0.0 and obj_dist < (0.4 + DIST_SENSOR)):
            control.stop_keyboard_input()
            # control.move_backward()
            control.send_cmd_vel( -1.0, 0.0)
            laserScan = lidar.checkScan()
            obj_dist = lidar.detect_obstacle_in_cone(laserScan,1.0,0,20)
            

# Automated test
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