#!/usr/bin/env python3.6
'''
This is the main controller for swarming behaviour of ChuhaLidarCamera.

To modify the swarming behaviour, change the swarm_control function.

The flow of code is as follows:
1) Enable all relevant devices and initialize global parameters.
2) Read and filter the LIDAR data
3) Categorize the LIDAR data
4) Obtain the positions of the neighbours from the LIDAR data
5) Swarm control

There is an optional graphing module which writes to the extra Display device provided.
The functionality to write to this Display is handled through the get_neighbours function.

There is an optional teleoperational module. To use it, comment out the call to swarm_control and
uncomment all the keyboard-related lines.
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
    '''
    To extract the detected shapes from the raw LIDAR data. This is done by comparing 
    the detected range with the default detected range for that layer when no object is there.
    To make the detection more strict, increase EPSILON. This will decrease false negatives but 
    will also decrease the detection range. To increase the detection range, decrease EPSILON.
    This may introduce more noise.
    '''
    theta_data = np.zeros((SIZES[1],)).tolist()
    for layer in range(SIZES[0]):
        for theta in range(SIZES[1]):
                point_range = imageArray[layer][theta]
                if(point_range < RANGES[layer]*(EPSILON)):
                    theta_data[theta] = point_range
    return theta_data
def get_theta_data_aligned(lidar, SIZES, RANGES, EPSILON):
    '''
    To get the LIDAR data in terms of right-handed r, theta. The robot's motion is 
    along the y axis but the LIDAR data's 'zero' of theta is at the +ve X axis.
    '''
    imageArray = np.array(lidar.getRangeImageArray()).T
    theta_data = lidar_filter(imageArray, SIZES,RANGES,EPSILON)
    theta_data_aligned = []
    for theta in range(len(theta_data)):
        new_theta =(+np.pi/2 -  2*np.pi*theta/SIZES[1])
        if(theta_data[theta] != 0):  
            theta_data_aligned.append((new_theta, theta_data[theta]))
    return theta_data_aligned
def get_theta_data_colored(theta_data_aligned, DELTA_THETA, DELTA_R):
    '''
    To categorize the data points on the basis of which robot they belong to. The logic being
    that points which are very close to each other are likely to belong to the same robot.
    If two robots are being detected where there ought to be one, increase DELTA_THETA or DELTA_R.
    If one robot is being detected where there are two, decrease DELTA_THETA or DELTA_R.
    '''
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
    return theta_data_colored
def get_neighbours(theta_data_colored,DISPLAY_SCALING_FACTOR=None,colors=None, grapher=None, shouldGraph=False):
    '''
    To get the (x,y) of the neighbours of a robot (and, optionally, to plot to the relevant grapher).
    '''
    neighbours_positions_x = []
    neighbours_positions_y = []
    if(shouldGraph):
        grapher.clear()
    color_index = 0
    for obstacle in theta_data_colored:
        if(len(obstacle) > 0):
            #PLOTTING ALL POINTS
            x_data = []
            y_data = []
            for theta, r in obstacle:
                x_data.append(r*np.cos(theta))
                y_data.append(r*np.sin(theta))
            x_mean = sum(x_data)/len(x_data)
            y_mean = sum(y_data)/len(y_data)
            neighbours_positions_x.append(x_mean)
            neighbours_positions_y.append(y_mean)
            if(shouldGraph):
                grapher.drawPointCenter(0, 0, size=5, color=0xFFFFFF)
                grapher.lidar_plot_without_clearing(x_data, y_data, DISPLAY_SCALING_FACTOR, color=colors[color_index])
                color_index = (color_index + 1)%len(colors)
    if(shouldGraph):
        grapher.lidar_plot_without_clearing(neighbours_positions_x, neighbours_positions_y,DISPLAY_SCALING_FACTOR, size=10, color=0x808080)    
    return neighbours_positions_x, neighbours_positions_y
def swarm_control(neighbours_positions_x, neighbours_positions_y, left_motor, right_motor, VELOCITY):
    '''
    The swarming behaviour is driven through this function. Modify it for different behaviour.
    '''
    LINEAR_FACTOR = 0.8*VELOCITY
    ANGULAR_FACTOR = 0.2*VELOCITY/np.pi
    if(len(neighbours_positions_x) > 0):
            mean_x = sum(neighbours_positions_x)/len(neighbours_positions_x)
            mean_y = sum(neighbours_positions_y)/len(neighbours_positions_y)
            angle = np.arctan2(mean_x, mean_y)
            distance = np.sqrt(mean_x**2 + mean_y**2)
            left_motor.setVelocity(distance*LINEAR_FACTOR + angle*ANGULAR_FACTOR)
            right_motor.setVelocity(distance*LINEAR_FACTOR - angle*ANGULAR_FACTOR)
def teleoperation(keyboard, left_motor, right_motor, VELOCITY):
    '''
    To teleoperate the robot using the keyboard's WASD keys.
    '''
    ascii_val = keyboard.getKey()
    if(ascii_val == -1):
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
    else:
        key = chr(ascii_val).lower()
        if(key == 'w'):
            left_motor.setVelocity(VELOCITY)
            right_motor.setVelocity(VELOCITY)
        elif(key == 's'):
            left_motor.setVelocity(-VELOCITY)
            right_motor.setVelocity(-VELOCITY)
        elif(key == 'a'):
            left_motor.setVelocity(-VELOCITY)
            right_motor.setVelocity(VELOCITY)
        elif(key == 'd'):
            left_motor.setVelocity(VELOCITY)
            right_motor.setVelocity(-VELOCITY)

if __name__ == "__main__":    
    #ROBOT INITIALIZATION
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

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
        neighbours_positions_x, neighbours_positions_y = get_neighbours(theta_data_colored) #To get the x, y of the neighbours
        
        swarm_control(neighbours_positions_x, neighbours_positions_y, left_motor, right_motor, VELOCITY)
        #To control the behaviour of the robot based on the relative positions of its neighbours.
        
        # teleoperation(keyboard, left_motor, right_motor, VELOCITY)
        