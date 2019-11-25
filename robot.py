#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase

# Write your program here


gyro = GyroSensor(Port.S2)
sensor = UltrasonicSensor(Port.S1)

gyro.reset_angle(0)

left = Motor(Port.B)
right = Motor(Port.C)
robot = DriveBase(left, right, 56, 114)

print(gyro.angle())

robot.drive(100, 180)
wait(500)
print(gyro.angle())
