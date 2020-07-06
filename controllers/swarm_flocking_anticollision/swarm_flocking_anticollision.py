#!/usr/bin/env python3.6

from controller import Robot, Motor, Lidar, Display, Keyboard
import numpy as np
def swarm_control_anticollide(neighbours_positions_x, neighbours_positions_y, left_motor, right_motor, VELOCITY, name):
    LINEAR_FACTOR = 3*VELOCITY
    ANGULAR_FACTOR = 6*VELOCITY
    FILTER_DISTANCE = 0.10
    COLLISION_WEIGHT = 10
    FOLLOWING_WEIGHT = 15
    n = len(neighbours_positions_x)
    chosen_x = 0
    chosen_y = 0
    if(n > 0):
        for i in range(n):
            curr_x, curr_y = neighbours_positions_x[i], neighbours_positions_y[i]
            if(np.sqrt(curr_x**2 +curr_y**2) < FILTER_DISTANCE):
                if(curr_y > 0):
                    new_x= -abs(curr_x)/curr_x*abs(COLLISION_WEIGHT/(curr_y + 0.000001*np.random.rand(1)))
                    new_y= -abs(COLLISION_WEIGHT/(curr_x + 0.000001*np.random.rand(1)))
                else:
                    new_x = -COLLISION_WEIGHT/(curr_x + 0.000001*np.random.rand(1))
                    new_y= -COLLISION_WEIGHT/(curr_y + 0.000001*np.random.rand(1))
                chosen_x += new_x
                chosen_y += new_y
            else:
                chosen_x += FOLLOWING_WEIGHT*curr_x
                chosen_y += FOLLOWING_WEIGHT*curr_y
        chosen_x = (chosen_x)/n
        chosen_y = (chosen_y)/n
        angle = np.arctan2(chosen_x, chosen_y)
        distance = min(np.sqrt(chosen_x**2 + chosen_y**2), 10)
        left_velocity = np.sqrt(distance)*LINEAR_FACTOR + angle*ANGULAR_FACTOR*abs(angle)*abs(angle)
        right_velocity = np.sqrt(distance)*LINEAR_FACTOR - angle*ANGULAR_FACTOR*abs(angle)*abs(angle)
        while(abs(left_velocity) > VELOCITY or abs(right_velocity) > VELOCITY):
            left_velocity = left_velocity/2
            right_velocity = right_velocity/2
        left_motor.setVelocity(float(left_velocity))
        right_motor.setVelocity(float(right_velocity))
if __name__ == "__main__":
    import sys, os
    sys.path.insert(0, os.path.abspath('..'))
from swarm_basic_flocking.swarm_basic_flocking import *
if __name__ == "__main__":    
    #ROBOT INITIALIZATION
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    name = robot.getName()
    #LIDAR INITIALIZATION
    lidar = robot.getLidar("lidar")
    lidar.enable(timestep)
    ranges_str = "1.13114178 0.85820043 0.57785118 0.43461093 0.38639969 0.31585345 0.2667459 0.23062678 0.21593061 0.19141567 0.17178488 0.15571462 0.14872716 0.13643947 0.12597121 0.11696267"
    RANGES = [float(i) for i in ranges_str.split(' ')]
    SIZES = (16, 512)
    EPSILON = 0.6

    #GRAPHER INITIALIZATION
    DISPLAY_SIZE = (1024, 1024)
    DISPLAY_SCALING_FACTOR = 1024/RANGES[0]
    display = robot.getDisplay("extra_display")
    grapher = Grapher(display)
    colors = [0xFF0000, 0x800000, 0xFFFF00, 0x808000, 0x00FF00,0x008000, 0x00FFFF, 0x008080, 0x0000FF, 0x000080, 0xFF00FF, 0x800080]
    
    #MOTOR INITIALIZATION
    VELOCITY = 60
    left_motor = robot.getMotor("left motor")
    right_motor = robot.getMotor("right motor")
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    
    #KEYBOARD INITIALIZATION
    # keyboard = Keyboard()
    # keyboard.enable(timestep)
    # print("Select the 3D window and control the robot using the W, A, S, D keys.")

    #NEIGHBOURS DETECTION INITIALIZATION
    DELTA_THETA = 0.1
    DELTA_R = 0.02

    while robot.step(timestep) != -1:
        theta_data_aligned = get_theta_data_aligned(lidar, SIZES, RANGES, EPSILON) #To read and filter the LIDAR data
        theta_data_colored = get_theta_data_colored(theta_data_aligned, DELTA_THETA, DELTA_R) #To categorize the LIDAR points into groups
        neighbours_positions_x, neighbours_positions_y = get_neighbours(theta_data_colored,DISPLAY_SCALING_FACTOR, colors, grapher, shouldGraph=False) #To get the x, y of the neighbours
        swarm_control_anticollide(neighbours_positions_x, neighbours_positions_y, left_motor, right_motor, VELOCITY, name)