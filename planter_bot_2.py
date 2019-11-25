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

sun_arm = Motor(Port.D, Direction.CLOCKWISE)

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
    


def sun_arm_move_down(angle):
    sun_arm.reset_angle(0)
    sun_arm.run_target(10, angle)

def sun_arm_move_up(angle):
    sun_arm.reset_angle(0)
    sun_arm.run_target(-10, -angle)

def sun_test():
    for j in range(3):
        for i in range(3):
            #test
            left_turn(120)
            left.run(0)
            right.run(0)
            wait(1000)
        sun_arm_move_down(20)
    sun_arm_move_up(60)
    brick.beep(1000,500)

sun_test()