#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# EV3 initialiseres
ev3 = EV3Brick()

# Robottens motorer initialiseres
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)

# En drivebase for robotten initialiseres
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=124)

# Afstandssensor og farvesensor initialiseres
distanceSensor = UltrasonicSensor(Port.S4)
colorSensor = ColorSensor(Port.S3)
gyroSensor = GyroSensor(Port.S2)
touchSensor = TouchSensor(Port.S1)

# Funktioner som tjekker farvereflektion og afstand initialiseres
def checkColor():
    return distanceSensor.reflection()

def checkDist():
    return colorSensor.distance()

def checkAngle():
    return gyroSensor.angle()



