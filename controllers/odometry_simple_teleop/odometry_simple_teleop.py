#!/usr/bin/env python3.6

from controller import Robot, Motor, PositionSensor, Keyboard, DistanceSensor, InertialUnit
import numpy as np
#INITIALIZATION CODE HERE.

VELOCITY = 30
WHEEL_RADIUS = 0.0075
#ROBOT INITIALIZATION
robot = Robot()
timestep = int(robot.getBasicTimeStep())


#MOTOR AND ENCODER INITIALIZATION
left_motor = robot.getMotor("left motor")
right_motor = robot.getMotor("right motor")

left_motor_sensor = left_motor.getPositionSensor()
right_motor_sensor = right_motor.getPositionSensor()

left_motor_sensor.enable(timestep)
right_motor_sensor.enable(timestep)

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

#KEYBOARD INITIALIZATION
keyboard = Keyboard()
keyboard.enable(timestep)
print("Select the 3D window and control the robot using the W, A, S, D keys.")


#INERTIAL UNIT INITIALIZATION
inertial_unit = robot.getInertialUnit("inertial unit")
inertial_unit.enable(timestep)

#ODOMETRY INITIALIZATION
_, _, alpha = inertial_unit.getRollPitchYaw()
x = 0 
z = 0
DTHETA_L = 0
DTHETA_R = 0
THETA_L_PREV = left_motor_sensor.getValue()
THETA_R_PREV = right_motor_sensor.getValue()
distance = 0

while robot.step(timestep) != -1:
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
        
         #HANDLING INERTIAL UNIT BEHAVIOUR
        _, _, alpha = inertial_unit.getRollPitchYaw()
        DTHETA_L = left_motor_sensor.getValue() - THETA_L_PREV
        DTHETA_R = right_motor_sensor.getValue() - THETA_R_PREV
        THETA_L_PREV += DTHETA_L
        THETA_R_PREV += DTHETA_R
        if(key == 'w' or key == 's'):
            DTHETA = (DTHETA_L + DTHETA_R)/2
            x += WHEEL_RADIUS*DTHETA*np.sin(alpha)
            z += WHEEL_RADIUS*DTHETA*np.cos(alpha)
            distance += abs(WHEEL_RADIUS*DTHETA)
        print(x, z, alpha, distance)