#!/usr/bin/env python3.6

from controller import Robot, Motor, PositionSensor, Keyboard, Lidar, Display
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

    def lidar_plot(self, x_data, y_data, DISPLAY_SCALING_FACTOR):
        x_s = []
        y_s = []
        for i in range(len(x_data)):
            x_s.append(int(x_data[i]*DISPLAY_SCALING_FACTOR))
            y_s.append(int(y_data[i]*DISPLAY_SCALING_FACTOR))
        self.clear()
        self.drawPointCenter(0, 0, color=0xFF0000)
        self.drawPointsListCenter(x_s, y_s, size=5)

def lidar_filter(imageArray, SIZES,RANGES,EPSILON):
    theta_data = np.zeros((SIZES[1],)).tolist()
    for layer in range(SIZES[0]):
        for theta in range(SIZES[1]):
                point_range = imageArray[layer][theta]
                if(point_range < RANGES[layer]*(EPSILON)):
                    theta_data[theta] = point_range
    return theta_data
    

#INITIALIZATION CODE HERE.
VELOCITY = 30
SIZES = (16, 512)
ranges_str = "1.13114178 0.85820043 0.57785118 0.43461093 0.38639969 0.31585345 0.2667459 0.23062678 0.21593061 0.19141567 0.17178488 0.15571462 0.14872716 0.13643947 0.12597121 0.11696267"
RANGES = [float(i) for i in ranges_str.split(' ')]
EPSILON = 0.5
DISPLAY_SIZE = (1024, 1024)
DISPLAY_SCALING_FACTOR = 0.3*1024/RANGES[0]
#ROBOT INITIALIZATION
robot = Robot()
timestep = int(robot.getBasicTimeStep())
PLOT_UPDATE_RATE = 1    


#MOTOR AND ENCODER INITIALIZATION
left_motor = robot.getMotor("left motor")
right_motor = robot.getMotor("right motor")

left_motor_sensor = left_motor.getPositionSensor()
right_motor_sensor = right_motor.getPositionSensor()

left_motor_sensor.enable(timestep)
right_motor_sensor.enable(timestep)

left_motor_init = left_motor_sensor.getValue()
right_motor_init = right_motor_sensor.getValue()

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

#KEYBOARD INITIALIZATION
keyboard = Keyboard()
keyboard.enable(timestep)
print("Select the 3D window and control the robot using the W, A, S, D keys.")

#LIDAR INITIALIZATION
lidar = robot.getLidar("lidar")
lidar.enable(timestep)


#DISPLAY INITIALIZATION
i = 0
display = robot.getDisplay("extra_display")
grapher = Grapher(display)
while robot.step(timestep) != -1:
    i += 1
    #HANDLING LIDAR IMAGE ACQUISITION
    imageArray = np.array(lidar.getRangeImageArray()).T
    
    #HANDLING KEYBOARD BEHAVIOUR
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
            #PLOTTING LIDAR DATA
    if(i % PLOT_UPDATE_RATE == 0):
        theta_data = lidar_filter(imageArray, SIZES,RANGES,EPSILON)
        x_data = []
        y_data = []
        for theta in range(len(theta_data)):
            x_data.append(-theta_data[theta]*np.cos(2*np.pi*theta/len(theta_data) + np.pi/2))
            y_data.append(theta_data[theta]*np.sin(2*np.pi*theta/len(theta_data) + np.pi/2))
        grapher.lidar_plot(x_data, y_data, DISPLAY_SCALING_FACTOR)
