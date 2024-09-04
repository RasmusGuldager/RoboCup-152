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

def colorControl():
    while True:
        if touchSensor.pressed() and colorStage == 1:
            white = checkColor()
            ev3.spreaker.WHITE()
            colorStage += 1
        elif touchSensor.pressed() and colorStage == 2:
            grey = checkColor()
            ev3.speaker.GREY()
            colorStage += 1
        elif touchSensor.pressed() and colorStage == 3:
            black = checkColor()
            ev3.speaker.BLACK()
            stage += 1
            stageControl()
            break

            

# Definér variabler
stage = 0
white = 0
grey = 0
black = 0
colorStage = 1
gyroSensor.reset_angle(0)

# Funktion til at styre hvilket stadie på banen robotten er nået til
def stageControl():
    if stage == 0:
        colorControl()


# Algoritme som får robotten til at følge en lige linje
def stage1():
    while True:
        color = checkColor()
        if color > 50:
            right_motor.run(-300)
            left_motor.run(-200)
        else:
            left_motor.run(-300)
            right_motor.run(-200)


stageControl() 
