#!/usr/bin/env python3.6
'''
NOTE: There are several places where 'grapher' has been disabled. This is to save on computing 
power, and these 'grapher' nodes are only necessary for data visualization.

NOTE2: The 'Keyboard' module is for debugging the LIDAR system by teleoperating the robot. When 
using the Keyboard for teleoperation, disable the regular function of the code by commenting it 
out.

I will have to refactor all this at some stage.
'''
from controller import Robot, Motor, Lidar, Display, Keyboard
import numpy as np
#PLOTTING FUNCTIONS
class Grapher:
    def __init__(self, display):
        '''
        display: The display you wish to use with this.
        '''
        self.display = display 
        self.width = display.getWidth()
        self.height = display.getHeight()
        self.defaultPointSize = int((self.height + self.width)/200)
    def drawPointCorner(self, x, y, size=None, color=0x00FFFF):
        '''
        Draws square points wrt the top right corner. The size is in pixels.
        '''
        if(size is None):
            size = self.defaultPointSize
        self.display.setColor(color)
        for row in range(max(0, y - size//2), min(self.height, y + size//2)):
            for column in range(max(0, x - size//2), min(self.width, x + size//2)):
                self.display.drawPixel(column, row)
    def drawPointCenter(self, x, y, size=None, color=0x00FFFF):
        '''
        Draws square points taking the center (width//2, height//2) to be 0, 0.
        '''
        self.drawPointCorner(x + self.width//2, self.height//2-y, size, color)
    def drawPointsListCenter(self, x_s, y_s, size = None, color=0x00FFFF):
        '''
        Draws points from two lists. If len_x != len_y, the trailing of x or y are truncated.        
        '''
        length = min(len(x_s), len(y_s))
        for i in range(length):
            self.drawPointCenter(x_s[i], y_s[i], size, color)

    def clear(self, color=0x000000):
        self.display.setColor(color)
        self.display.fillRectangle(0, 0, self.width, self.height)

    def lidar_plot(self, x_data, y_data, DISPLAY_SCALING_FACTOR, color=0x00FFFF, size=5):
        self.clear()
        self.lidar_plot_without_clearing(x_data, y_data, DISPLAY_SCALING_FACTOR, color, size=5)
    def lidar_plot_without_clearing(self, x_data, y_data, DISPLAY_SCALING_FACTOR, color=0x00FFFF, size=5):
        x_s = []
        y_s = []
        for i in range(len(x_data)):
            x_s.append(int(x_data[i]*DISPLAY_SCALING_FACTOR))
            y_s.append(int(y_data[i]*DISPLAY_SCALING_FACTOR))
        # self.drawPointCenter(0, 0, color=0xFF0000)
        self.drawPointsListCenter(x_s, y_s, size=size,color=color)
def lidar_filter(imageArray, SIZES,RANGES,EPSILON):
    theta_data = np.zeros((SIZES[1],)).tolist()
    for layer in range(SIZES[0]):
        for theta in range(SIZES[1]):
                point_range = imageArray[layer][theta]
                if(point_range < RANGES[layer]*(EPSILON)):
                    theta_data[theta] = point_range
    return theta_data

#INITIALIZATION CODE HERE.
VELOCITY = 60

#ROBOT INITIALIZATION
robot = Robot()
timestep = int(robot.getBasicTimeStep())

#LIDAR AND PLOTTING INITIALIZATION
SIZES = (16, 512)
ranges_str = "1.13114178 0.85820043 0.57785118 0.43461093 0.38639969 0.31585345 0.2667459 0.23062678 0.21593061 0.19141567 0.17178488 0.15571462 0.14872716 0.13643947 0.12597121 0.11696267"
RANGES = [float(i) for i in ranges_str.split(' ')]
EPSILON = 0.6
DISPLAY_SIZE = (1024, 1024)
DISPLAY_SCALING_FACTOR = 1024/RANGES[0]
lidar = robot.getLidar("lidar")
lidar.enable(timestep)
# display = robot.getDisplay("extra_display")
# grapher = Grapher(display)

#MOTOR INITIALIZATION
left_motor = robot.getMotor("left motor")
right_motor = robot.getMotor("right motor")

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)
#KEYBOARD -- TEMPORARY
# keyboard = Keyboard()
# keyboard.enable(timestep)
# print("Select the 3D window and control the robot using the W, A, S, D keys.")

#NEIGHBOURS DETECTION INITIALIZATION
DELTA_THETA = 0.1
DELTA_R = 0.02
# colors = [0xFF0000, 0x800000, 0xFFFF00, 0x808000, 0x00FF00,0x008000, 0x00FFFF, 0x008080, 0x0000FF, 0x000080, 0xFF00FF, 0x800080]

#SWARM CONTROL INITIALIZATION
LINEAR_FACTOR = 0.8*VELOCITY
ANGULAR_FACTOR = 0.2*VELOCITY/np.pi
while robot.step(timestep) != -1:

    #ACQUIRING, FILTERING AND ALIGNING LIDAR DATA
    imageArray = np.array(lidar.getRangeImageArray()).T
    theta_data = lidar_filter(imageArray, SIZES,RANGES,EPSILON)
    theta_data_aligned = []
    for theta in range(len(theta_data)):
        new_theta =(+np.pi/2 -  2*np.pi*theta/512)
        if(theta_data[theta] != 0):  
            theta_data_aligned.append((new_theta, theta_data[theta]))

    #CATEGORIZING LIDAR DATA
    color_index = 0
    theta_data_colored = [[]]
    theta_prev = None 
    r_prev = None
    for theta, r in theta_data_aligned:
        if(not(theta_prev is None and r_prev is None)): 
            if(not (abs(theta_prev - theta) < DELTA_THETA and abs(r_prev - r) < DELTA_R/r)):
                theta_data_colored.append([])
        theta_data_colored[-1].append((theta, r))
        theta_prev = theta 
        r_prev = r

    if(len(theta_data_colored[0]) > 0 and len(theta_data_colored) > 1):
        theta_prev, r_prev = theta_data_colored[0][0]
        theta, r = theta_data_colored[-1][-1]
        if(abs(theta_prev - theta - 2*np.pi) < DELTA_THETA and abs(r_prev - r) < DELTA_R/r):
            last_category = theta_data_colored.pop()
            theta_data_colored[0].extend(last_category)
    #PLOTTING LIDAR DATA
    # color_index = 0
    # grapher.clear()
    neighbours_positions_x = []
    neighbours_positions_y = []
    for obstacle in theta_data_colored:
        if(len(obstacle) > 0):
            #PLOTTING ALL POINTS
            x_data = []
            y_data = []
            for theta, r in obstacle:
                x_data.append(r*np.cos(theta))
                y_data.append(r*np.sin(theta))
            # grapher.drawPointCenter(0, 0, size=5, color=0xFFFFFF)
            # grapher.lidar_plot_without_clearing(x_data, y_data, DISPLAY_SCALING_FACTOR, color=colors[color_index])
            
            x_mean = sum(x_data)/len(x_data)
            y_mean = sum(y_data)/len(y_data)
            neighbours_positions_x.append(x_mean)
            neighbours_positions_y.append(y_mean)
            # color_index = (color_index + 1)%len(colors)
    # grapher.lidar_plot_without_clearing(neighbours_positions_x, neighbours_positions_y,DISPLAY_SCALING_FACTOR, size=10, color=0x808080)
    #KEYBOARD -- TEMPORARY
    # ascii_val = keyboard.getKey()
    # if(ascii_val == -1):
    #     left_motor.setVelocity(0)
    #     right_motor.setVelocity(0)
    # else:
    #     key = chr(ascii_val).lower()
    #     if(key == 'w'):
    #         left_motor.setVelocity(VELOCITY)
    #         right_motor.setVelocity(VELOCITY)
    #     elif(key == 's'):
    #         left_motor.setVelocity(-VELOCITY)
    #         right_motor.setVelocity(-VELOCITY)
    #     elif(key == 'a'):
    #         left_motor.setVelocity(-VELOCITY)
    #         right_motor.setVelocity(VELOCITY)
    #     elif(key == 'd'):
    #         left_motor.setVelocity(VELOCITY)
    #         right_motor.setVelocity(-VELOCITY)

    #CONTROL OF ROBOT BASED ON POSITIONS OF NEIGHBOURS
    if(len(neighbours_positions_x) > 0):
        mean_x = sum(neighbours_positions_x)/len(neighbours_positions_x)
        mean_y = sum(neighbours_positions_y)/len(neighbours_positions_y)
        angle = np.arctan2(mean_x, mean_y)
        distance = np.sqrt(mean_x**2 + mean_y**2)
        left_motor.setVelocity(distance*LINEAR_FACTOR + angle*ANGULAR_FACTOR)
        right_motor.setVelocity(distance*LINEAR_FACTOR - angle*ANGULAR_FACTOR)