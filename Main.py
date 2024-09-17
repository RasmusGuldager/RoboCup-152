#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time


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
gyroSensor = GyroSensor(Port.S3, Direction.COUNTERCLOCKWISE)
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
    global grey
    global colorStage
    global stage
    while True:
        if touchSensor.pressed() and colorStage == 1:
            white = CheckColor()
            ev3.speaker.say("White " + str(white))
            colorStage += 1
        elif touchSensor.pressed() and colorStage == 2:
            break
    grey = CheckColor()
    ev3.speaker.say("Grey " + str(grey))
    stage += 1
    wait(1500)
    StageControl()

def FollowLine(speed1, speed2):
    global white
    global grey
    global stage
    global right_motor
    global left_motor
    while True:
        color = CheckColor()
        if color >= (white + grey) / 2:
            right_motor.run(speed1)
            left_motor.run(speed2)
        elif color < (white + grey) / 2 and color > 15:
            right_motor.run(speed2)
            left_motor.run(speed1)
        else: break
    robot.stop()
    stage += 1
    StageControl()


def AdjustGyro(speed1, speed2):
    global white
    global grey
    global stage
    global right_motor
    global left_motor
    start_time = time.time()
    while time.time() - start_time < 4:
        color = CheckColor()
        if color >= (white + grey) / 2:
            right_motor.run(-40)
            left_motor.run(-36)
        elif color < (white + grey) / 2 and color > 15:
            right_motor.run(-36)
            left_motor.run(-40)
    robot.stop()
    gyroSensor.reset_angle(0)



def TurnToAngle(angle, speed, epsilon):
    
    if angle < gyroSensor.angle():
        left_motor.run(-speed)
        right_motor.run(speed)
        while angle < gyroSensor.angle():
            pass
    elif angle > gyroSensor.angle():
        left_motor.run(speed)
        right_motor.run(-speed)
        while angle > gyroSensor.angle():
            pass
    
    left_motor.hold()
    right_motor.hold()
 
    wait(150)
    if angle > gyroSensor.angle() + epsilon:
        TurnToAngle(angle, speed * 0.33, epsilon)
    elif angle < gyroSensor.angle() - epsilon:
        TurnToAngle(angle, speed * 0.33, epsilon)


def ApproachLineStraight(speed):
    global grey
    robot.drive(speed, 0)
    while CheckColor() > grey + 15:
        pass
    robot.stop()


def RunForkliftUp():
    small_motor.run_until_stalled(300, Stop.HOLD, 90)


def RunForkliftDown():
    small_motor.run_until_stalled(-300, Stop.HOLD, 70)

            

# Definér variabler
stage = 0
white = 80
grey = 40
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
    AdjustGyro()
    FollowLine(-450, -300)

# Del to af brudt streg
def Stage2():
    print(2, CheckAngle())
    TurnToAngle(-30, 200, 5)
    ApproachLineStraight(-200)
    FollowLine(-300, -450)

# 180 grader sving
def Stage3():
    print(3, CheckAngle())
    TurnToAngle(30, 200, 5)
    ApproachLineStraight(-200)
    FollowLine(-450, -300)

# Flyt flaske
def Stage4():
    print(4, CheckAngle())
    robot.straight(-300)
    #robot.turn(CheckAngle() + 90)
    robot.stop()
    TurnToAngle(-270, 200, 2)
    robot.drive(-200, 0)
    while CheckDist() > 120:
        pass
    robot.stop()
    RunForkliftUp()
    robot.straight(-250)
    RunForkliftDown()
    robot.straight(200)
    robot.stop()
    TurnToAngle(-90, 200, 2)
    robot.straight(-150)
    robot.stop()
    TurnToAngle(-180, 200, 2)
    FollowLine(-450, -300)


# Venstresving over til vippen
def Stage5():
    print(5, CheckAngle())
    robot.straight(-100)
    robot.stop()
    FollowLine(-450, -300)

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
    FollowLine(-450, -300)

# Venstresving over til dartskive
def Stage8():
    print(8, CheckAngle())
    robot.straight(-100)
    robot.turn(90)
    robot.stop()
    FollowLine(-450, -400)

# Dartskive
def Stage9():
    '''
    global stage
    stage += 1
    StageControl()
    '''
    robot.turn()
    robot.stop()
    FollowLine(-300, -450)
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
    robot.turn(-45)
    robot.drive(-150, 10)
    while True:
        color = CheckColor()
        if color < white - 20:
            robot.turn(-20)
            robot.stop()
            FollowLine(-450, -300)
            break

# Zig-zag rundt om muren
def Stage12():
    print(12, CheckAngle())
    robot.straight(-100)
    robot.turn(-85)
    robot.drive(-150, 12)
    while True:
        color = CheckColor()
        if color < white - 20:
            robot.turn(-30)
            robot.stop()
            FollowLine(-450, -300)
            break

# Rundt om flasken #2
def Stage13():
    print(13, CheckAngle())
    robot.straight(-100)
    robot.turn(-55)
    robot.drive(-100, 9)
    while True:
        color = CheckColor()
        if color < white - 20:
            robot.straight(-150)
            robot.stop()
            #AdjustGyro()
            FollowLine(-450, -300)
            break

# Landingsbane
def Stage14():
    print(14, CheckAngle())
    robot.turn(25)
    #robot.turn(CheckAngle() - 180)
    while True:
        distance = CheckDist()
        robot.drive(100)
        if distance > 1250:
            robot.stop()
            break


StageControl()