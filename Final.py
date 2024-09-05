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
def CheckColor():
    return colorSensor.reflection()

def CheckDist():
    return distanceSensor.distance()

def CheckAngle():
    return gyroSensor.angle()

def ColorControl():
    global white
    global black
    global colorStage
    global stage
    while True:
        if touchSensor.pressed() and colorStage == 1:
            white = CheckColor()
            ev3.speaker.say("White " + str(white))
            colorStage += 1
        elif touchSensor.pressed() and colorStage == 2:
            break
    black = CheckColor()
    ev3.speaker.say("Black " + str(black))
    stage += 1
    wait(5000)
    StageControl()

def FollowLine():
    global white
    global black
    global stage
    global right_motor
    global left_motor
    while True:
        color = CheckColor()
        if color >= white - 10:
            right_motor.run(-300)
            left_motor.run(-200)
        elif color < white - 10 and color > black + 10:
            right_motor.run(-200)
            left_motor.run(-300)
        else: break
    robot.stop()
    stage += 1
    StageControl()
            

# Definér variabler
stage = 0
white = 75
black = 6
colorStage = 1
gyroSensor.reset_angle(0)

# Funktion til at styre hvilket stadie på banen robotten er nået til
def StageControl():
    if stage == 0:
        ColorControl()
    elif stage == 1:
        Stage1()
    elif stage == 2:
        Stage2()
    elif stage == 3:
        Stage3()


def Stage1():
    FollowLine()

def Stage2():
    robot.turn(-30)
    robot.straight(-400)
    robot.turn(30)
    robot.stop()
    FollowLine()


def Stage3():
    robot.turn(30)
    robot.straight(-400)
    robot.turn(-30)
    robot.stop()
    FollowLine()



StageControl()