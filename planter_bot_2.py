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

#define motors
left = Motor(Port.B)
right = Motor(Port.C)
robot = DriveBase(left, right, 56, 114)


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
    if (gyro.angle() < - angle):
        OVERSHOOT = (gyro.angle()-angle)
        left_turn(OVERSHOOT)
        
def left_turn_precise(angle):
    #robot will rotate to specified angle in a counter clockwise direction 
    gyro.reset_angle(0)

    while (gyro.angle() < angle):
        left.run(-100)
        right.run(100)
    if (gyro.angle() >  angle):
        OVERSHOOT = (angle-gyro.angle())
        right_turn(OVERSHOOT)
    


#left_turn(90)
#right_turn(90)
right_turn_precise(90)
left_turn_precise(90)