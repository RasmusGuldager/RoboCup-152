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
right_motor = Motor(Port.A)
left_motor = Motor(Port.B)
small_motor = Motor(Port.C)

# En drivebase for robotten initialiseres
robot = DriveBase(left_motor, right_motor, wheel_diameter=70, axle_track=135)

# Afstandssensor og farvesensor initialiseres
colorSensor = ColorSensor(Port.S1)
distanceSensor = UltrasonicSensor(Port.S2)
gyroSensor = GyroSensor(Port.S3)
touchSensor = TouchSensor(Port.S4)

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
white = 85
black = 6
colorStage = 1

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

# Del ét af brudt streg
def Stage1():
    gyroSensor.reset_angle(0)
    FollowLine(-300, -200)

# Del to af brudt streg
def Stage2():
    print(2, CheckAngle())
    robot.turn(-30)
    robot.straight(-600)
    robot.turn(30)
    robot.stop()
    FollowLine(-300, -200)

# 180 grader sving
def Stage3():
    print(3, CheckAngle())
    robot.turn(30)
    robot.straight(-400)
    robot.turn(-30)
    robot.stop()
    FollowLine(-300, -200)

# Flyt flaske
def Stage4():
    print(4, CheckAngle())
    robot.straight(-100)
    robot.stop()
    FollowLine(-300, -250)
    '''
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
    '''


# Venstresving over til vippen
def Stage5():
    print(5, CheckAngle())
    robot.straight(-100)
    robot.stop()
    FollowLine(-300, -200)

# Vippen
def Stage6():
    global stage
    stage += 1
    StageControl()

# Parallelle streger
def Stage7():
    print(7, CheckAngle())
    robot.straight(-100)
    robot.stop()
    FollowLine(-300, -200)

# Venstresving over til dartskive
def Stage8():
    print(8, CheckAngle())
    robot.straight(-100)
    robot.stop()
    FollowLine(-300, -200)

# Dartskive
def Stage9():
    global stage
    stage += 1
    StageControl()
"""
Løsning til dartskive:
Kun hvis ikke followLine kan udføres direkte fra sort linje
**Drej en bestemt vinkel
**Kør ligeud indtil followLine kan udføres

Afvent signal fra distancesensor med prædefineret afstand til flaske
Udfør protokol for at samle flasken op
Drej til en bestemt vinkel vha gyroskop
kør en prædefineret længde
sæt flasken
lav followLine igen
"""

# Fra dartskive over til rundt om flasken
def Stage10():
    global stage
    stage += 1
    StageControl()

# Rundt om flasken #1
def Stage11():
    print(11, CheckAngle())
    robot.turn(-30)
    robot.drive(-100, 5)
    while True:
        color = CheckColor()
        if color < white - 10:
            robot.turn(-30)
            robot.stop()
            FollowLine(-300, -200)
            break

# Zig-zag rundt om muren
def Stage12():
    print(12, CheckAngle())
    robot.turn(-85)
    robot.drive(-150, 12)
    while True:
        color = CheckColor()
        if color < white - 10:
            robot.turn(-30)
            robot.stop()
            FollowLine(-300, -200)
            break

# Rundt om flasken #2
def Stage13():
    print(13, CheckAngle())
    robot.turn(-35)
    robot.drive(-100, 5)
    while True:
        color = CheckColor()
        if color < white - 10:
            robot.straight(-150)
            robot.stop()
            FollowLine(-300, -200)
            break

# Landingsbane
def Stage14():
    print(14, CheckAngle())
    robot.turn(25)
    robot.straight(-1700)

StageControl()