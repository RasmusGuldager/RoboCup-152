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
left_motor = Motor(Port.B)
right_motor = Motor(Port.A)

# En drivebase for robotten initialiseres
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=124)

# Afstandssensor og farvesensor initialiseres
colorSensor = ColorSensor(Port.S1)
gyroSensor = GyroSensor(Port.S2)
touchSensor = TouchSensor(Port.S3)
distanceSensor = UltrasonicSensor(Port.S4)

# Funktioner som tjekker farvereflektion, afstand og rumlig orientering initialiseres
def CheckColor():
    return colorSensor.reflection()

def CheckDist():
    return distanceSensor.distance()

def CheckAngle():
    return gyroSensor.angle()


# Funktion som bruges til at kalibrere robottens farvesensor efter farverne på banen
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

def FollowLine(speed1, speed2):
    global white
    global black
    global stage
    global right_motor
    global left_motor
    while True:
        color = CheckColor()
        if color >= white - 10:
            right_motor.run(speed1)
            left_motor.run(speed2)
        elif color < white - 10 and color > black + 10:
            right_motor.run(speed2)
            left_motor.run(speed1)
        else: break
    robot.stop()
    stage += 1
    StageControl()

def FindBottle():
    while True:
        distance = CheckDistance()
        if distance < 500:
            robot.stop()
            break
        else: robot.drive(200, 0)
            

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
    elif stage == 4:
        Stage4()
    elif stage == 5:
        Stage5()
    elif stage == 6:
        Stage6()
    elif stage == 7:
        Stage7()
    elif stage == 8:
        Stage8()
    elif stage == 9:
        Stage9()
    elif stage == 10:
        Stage10()
    elif stage == 11:
        Stage11()
    elif stage == 12:
        Stage12()
    elif stage == 13:
        Stage13()
    elif stage == 14:
        Stage14()

# 180 grader sving
def Stage3():
    robot.turn(30)
    robot.straight(-400)
    robot.turn(-30)
    robot.stop()
    FollowLine(-300, -200)

# Flyt flaske
def Stage4():
    robot.turn(CheckAngle() - 180)
    robot.straight(-200)
    robot.turn(CheckAngle() - 90)
    robot.straight(-200)
    FindBottle()
    #Saml flasken op og kør den over stregen
    robot.turn(CheckAngle() - 270)
    robot.straight(-500)
    robot.turn(-45)
    FollowLine()

StageControl()

            