#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=124)

# Afstandssensor og farvesensor
Afstandssensor = UltrasonicSensor(Port.S4)
Farvesensor = ColorSensor(Port.S3)

def checkColor():
    return Farvesensor.reflection()

def checkDist():
    return Afstandssensor.distance()


angle = 10
speed = 100


while True:
    color = checkColor()
    if color > 50:
        right_motor.run(-300)
        left_motor.run(-200)
    else:
        left_motor.run(-300)
        right_motor.run(-200)



