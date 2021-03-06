#!/usr/bin/env python3.6

from controller import Robot, Motor, PositionSensor, Keyboard, Lidar, LidarPoint

#INITIALIZATION CODE HERE.

VELOCITY = 30

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
lidar.enablePointCloud()

#FRONT CAMERA INITIALIZATION
front_camera = robot.getCamera("front_camera")
front_camera.enable(timestep)
#REAR CAMERA INITIALIZATION
rear_camera = robot.getCamera("rear_camera")
rear_camera.enable(timestep)
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
    #HANDLING LIDAR
    pointCloud = lidar.getPointCloud()
    #HANDLING FRONT CAMERA
    front_camera.getImageArray()
    #HANDLING REAR CAMERA
    rear_camera.getImageArray()