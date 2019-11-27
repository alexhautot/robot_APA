#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase


#define sensors
gyro = GyroSensor(Port.S2)
sensor = UltrasonicSensor(Port.S1)
colour = ColorSensor(Port.S3) 

#define motors
left = Motor(Port.B)
right = Motor(Port.C)
robot = DriveBase(left, right, 56, 114)

#sun_arm = Motor(Port.D, Direction.CLOCKWISE)
picker = Motor(Port.D, Direction.CLOCKWISE)


def forwardMotion(a):
    gyro.reset_angle(0)
    repeatMove = a
    for x in range(repeatMove):
        # start the robot driving
        # use a loop to wait for rotation sensor to reach 720
        repeatMove -= 1
        left.reset_angle(0)
        while left.angle() < 720:
            if gyro.angle() > 1:
                robot.drive(200, 20)
            elif gyro.angle() < -1:
                robot.drive(200, -20)
            else:
                robot.drive (200, 0)        
    robot.stop(Stop.BRAKE)


def left_turn(angle):
    #robot will rotate to specified angle in a counter clockwise direction 
    gyro.reset_angle(0)

    while (gyro.angle() < angle):
        left.run(-100)
        right.run(100)

def right_turn(angle):
    #robot will rotate to specified angle in a clockwise direction
    gyro.reset_angle(0)

    while (gyro.angle() >-angle):
        left.run(100)
        right.run(-100)

def right_turn_precise(angle):
    #robot will rotate to specified angle in a clockwise direction
    gyro.reset_angle(0)

    while (gyro.angle() >-angle):
        left.run(100)
        right.run(-100)
    left.run(0)
    right.run(0)
    wait(200)
    overshoot = (gyro.angle() + angle)
    print(overshoot)
    if (overshoot < -1):
        left_turn(-overshoot)
        
def left_turn_precise(angle):
    #robot will rotate to specified angle in a counter clockwise direction 
    gyro.reset_angle(0)
    while (gyro.angle() < angle):
        left.run(-100)
        right.run(100)
    left.run(0)
    right.run(0)
    wait(200)
    overshoot = (gyro.angle() - angle)
    print(overshoot)
    if (overshoot > 1):
        right_turn(overshoot)
    
def grid_walk(dist, n): 
    for i in range(n):
        for j in range(n):
            sun_test()
            forwardMotion(dist)
        if i % 2 == 0:
            left_turn_precise(90)
            forwardMotion(dist)
            left_turn_precise(90)
        else:
            right_turn_precise(90)
            forwardMotion(dist)
            wait(1000)
            right_turn_precise(90)


def sun_arm_move_down(angle):
    sun_arm.reset_angle(0)
    sun_arm.run_target(10, angle)

def sun_arm_move_up(angle):
    sun_arm.reset_angle(0)
    sun_arm.run_target(-10, -angle)



def sun_test():
    measurments = []
    measurement = colour.ambient()
    measurments.append(measurement)
    for j in range(1):
        sun_arm_move_down(20)
        for i in range(3):
            #test
            left_turn_precise(120)
            left.run(0)
            right.run(0)
            measurement = colour.ambient()
            measurments.append(measurement)
    sun_arm_move_up(20)
    f = open('measurements.csv', 'a+')
    for item in measurments:
        m_to_write = '%d' % (item) + ', '
        f.write(m_to_write)
    f.write('\n')
    f.close()

f = open('meaurements.csv', 'w')
f.close()
def pick_up():
    picker.run_target(10,-180)
    left.run(30)
    right.run(30)
    wait(200)
    left.run(0)
    right.run(0)
    picker.run_target(10,180)


right_turn_precise(90)
right_turn_precise(90)
right_turn_precise(90)
