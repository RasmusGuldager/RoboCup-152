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
    global stage
    colorStage = 1
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


def AdjustGyro(runtime):
    global white
    global grey
    global stage
    global right_motor
    global left_motor
    start_time = time.time()
    while time.time() - start_time < runtime:
        color = CheckColor()
        if color >= (white + grey) / 2:
            right_motor.run(-45)
            left_motor.run(-30)
        elif color < (white + grey) / 2 and color > 15:
            right_motor.run(-30)
            left_motor.run(-45)
    robot.stop()
    gyroSensor.reset_angle(0)



def TurnToAngle(angle, speed, epsilon):
    if angle < CheckAngle():
        left_motor.run(-speed)
        right_motor.run(speed)
        while angle < CheckAngle():
            wait(20)

    elif angle > CheckAngle():
        left_motor.run(speed)
        right_motor.run(-speed)
        while angle > CheckAngle():
            wait(20)
    
    left_motor.hold()
    right_motor.hold()
 
    wait(150)
    if angle > CheckAngle() + epsilon:
        TurnToAngle(angle, speed * 0.33, epsilon)
    elif angle < CheckAngle() - epsilon:
        TurnToAngle(angle, speed * 0.33, epsilon)


def ApproachLineStraight(speed):
    global grey
    robot.drive(speed, 0)
    while CheckColor() > grey + 15:
        pass
    robot.stop()


def RunForkliftUp(power):
    small_motor.run_until_stalled(300, Stop.HOLD, power)

def RunForkliftDown(power):
    small_motor.run_until_stalled(-300, Stop.HOLD, power)

def FindBottle():
    '''
    global right_motor
    global left_motor
    prev_dist = 0
    switch = False
    right_motor.run(speed1)
    left_motor.run(speed2)
    while True:
        distance = CheckDist()
        if distance > prev_dist and distance < 700:
            if switch:
                right_motor.run(speed2)
                left_motor.run(speed1)
                switch = False
            else:
                right_motor.run(speed1)
                left_motor.run(speed2)
                switch = True
            wait(100)
        prev_dist = distance
        wait(50)
        if distance < 90:
            break
    robot.stop()
    '''
    global right_motor
    global left_motor
    angles = []
    while True:
        distance = CheckDist()
        if distance < 300:
            ev3.speaker.beep()
            angles.append(CheckAngle())
            break
        wait(50)
        robot.turn(-2)
        wait(50)
    robot.turn(-50)
    wait(1000)
    while True:
        distance = CheckDist()
        if distance < 300:
            ev3.speaker.beep()
            angles.append(CheckAngle())
            break
        wait(50)
        robot.turn(2)
        wait(50)
    robot.stop()
    return (angles[0] + angles[1]) / 2


# Definér variabler
stage = 1
white = 80
grey = 40

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
    # Luk gribearm
    RunForkliftUp(40)
    AdjustGyro(2)
    FollowLine(-450, -400)

# Del to af brudt streg
def Stage2():
    robot.turn(-40)
    robot.stop()
    ApproachLineStraight(-200)
    FollowLine(-350, -450)

# 180 grader sving
def Stage3():
    robot.turn(30)
    robot.stop()
    ApproachLineStraight(-200)
    FollowLine(-450, -300)

# Flyt flaske
def Stage4():
    # Approach
    robot.straight(-270)
    robot.stop()
    TurnToAngle(-270, 200, 0.5)
    RunForkliftDown(45)
    robot.drive(-180, 0)
    '''
    robot.straight(-120)
    robot.stop()
    TurnToAngle(-230, 200, 2)
    angle_avg = FindBottle()
    TurnToAngle(angle_avg, 100, 1)
    robot.drive(-80, 0)
    while CheckDist() > 85:
        pass
    robot.stop()
    '''
    while CheckDist() > 85:
        wait(50)
    robot.stop()
    # Moving the bottle
    RunForkliftUp(90)
    robot.straight(-250)
    RunForkliftDown(65)
    robot.straight(200)
    robot.stop()
    # Væk fra flasken
    TurnToAngle(-90, 200, 2)
    robot.straight(-250)
    robot.stop()
    TurnToAngle(-180, 200, 2)
    RunForkliftUp(40)
    FollowLine(-450, -380)


# Venstresving over til vippen
def Stage5():
    global stage
    robot.straight(-300)
    robot.stop()
    TurnToAngle(-90, 200, 2)
    gyroSensor.reset_angle(0)
    while True:
        if CheckColor() > 15:
            robot.drive(-320, 0)
        else:
            stage += 1
            StageControl()

# Vippen
def Stage6():
    global white
    global grey
    global stage
    global right_motor
    global left_motor
    count = 0
    robot.straight(-100)
    robot.stop()
    start_time = time.time()
    while count < 0.5 or time.time() - start_time < 9:
        temp_time = time.time()
        color = CheckColor()
        if color >= (white + grey) / 2:
            right_motor.run(-330)
            left_motor.run(-300)
            count += time.time() - temp_time
        elif color < (white + grey) / 2 and color > 15:
            right_motor.run(-300)
            left_motor.run(-330)
            count = 0
    robot.stop()
    TurnToAngle(90, 200, 3)
    FollowLine(-450, -400)


# Parallelle streger
def Stage7():
    robot.straight(-100)
    robot.stop()
    FollowLine(-400, -300)

# Venstresving over til dartskive
def Stage8():
    robot.straight(-100)
    robot.stop()
    AdjustGyro(1)
    robot.straight(-200)
    robot.stop()
    TurnToAngle(90, 200, 3)
    FollowLine(-370, -350)

# Dartskive
def Stage9():
    AdjustGyro(3)
    #TurnToAngle(90, 200, 0.5)
    robot.straight(-540)
    robot.stop()
    TurnToAngle(120 - 90, 200, 2)
    RunForkliftDown(45)
    '''
    angle_avg = FindBottle()
    TurnToAngle(angle_avg, 100, 1)
    robot.drive(-80, 0)
    while CheckDist() > 85:
        pass
    robot.stop()
    '''
    robot.drive(-150, 0)
    while CheckDist() > 85:
        wait(50)
    robot.stop()

    RunForkliftUp(90)
    robot.straight(540)
    RunForkliftDown(65)
    robot.straight(200)
    robot.stop()
    TurnToAngle(230 - 90, 200, 3)
    RunForkliftUp(30)
    global stage
    stage += 1
    StageControl()


# Fra dartskive over til rundt om flasken
def Stage10():
    ApproachLineStraight(-200)
    count = 0
    start_time = time.time()
    while count < 0.5 or time.time() - start_time < 3:
        temp_time = time.time()
        color = CheckColor()
        if color >= (white + grey) / 2:
            right_motor.run(-300)
            left_motor.run(-450)
            count += time.time() - temp_time
        elif color < (white + grey) / 2 and color > 15:
            right_motor.run(-450)
            left_motor.run(-300)
            count = 0
    robot.stop()

    TurnToAngle(360 - 90, 200, 3)
    FollowLine(-450, -300)

# Rundt om flasken #1
def Stage11():
    robot.straight(-50)
    robot.stop()
    AdjustGyro(3)
    TurnToAngle(-45, 200, 3)
    robot.drive(-150, 12)
    while True:
        color = CheckColor()
        if color < white - 20:
            wait(1000)
            robot.turn(-80)
            robot.stop()
            break
    FollowLine(-440, -280)

# Zig-zag rundt om muren
def Stage12():
    robot.straight(-100)
    robot.stop()
    TurnToAngle(-260, 200, 3)
    robot.drive(-170, 15)
    while True:
        color = CheckColor()
        if color < white - 20:
            wait(200)
            robot.turn(-30)
            robot.stop()
            break
    FollowLine(-450, -300)

# Rundt om flasken #2
def Stage13():
    robot.straight(-100)
    TurnToAngle(-230, 200, 3)
    robot.drive(-100, 9)
    while True:
        color = CheckColor()
        if color < white - 20:
            robot.straight(-150)
            robot.stop()
            break
    FollowLine(-450, -300)

# Landingsbane
def Stage14():
    TurnToAngle(0, 200, 2)
    robot.straight(-20)
    robot.stop()
    AdjustGyro(4)
    while True:
        distance = CheckDist()
        robot.drive(200, CheckAngle())
        if distance < 1250:
            robot.stop()
            break



StageControl()