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
robot.settings(straight_acceleration = 250)

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

gyroDriftRate = 0
def CheckAngle():
    global isDrifting
    global gyroResetTime
    global gyroDriftRate

    if isDrifting == False:
        return gyroSensor.angle()
    else:
        antidrift = (time.time() - gyroResetTime) * gyroDriftRate
        return gyroSensor.angle() - antidrift



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
    #gyroSensor.reset_angle(0)
    ResetGyro(0)

gyroResetTime = 0
def ResetGyro(angle):
    global gyroResetTime

    gyroSensor.reset_angle(angle)
    gyroResetTime = time.time()


isDrifting = False
def TurnToAngleFailState(angle, speed, epsilon):
    global gyroDriftRate
    global isDrifting

    left_motor.hold()
    right_motor.hold()
    wait(200)

    startAngle = CheckAngle()
    #startMeasureTime = time.time()
    wait(1000)
    endAngle = CheckAngle()
    #endMeasureTime = time.time()

    #elapsedTime = endMeasureTime - startMeasureTime
    gyroDriftRate = endAngle - startAngle

    isDrifting = True
    TurnToAngle(angle,speed,epsilon)



isTurningToAngle = False
turnToAngleStartTime = 0
def TurnToAngle(angle, speed, epsilon):
    global isTurningToAngle
    global turnToAngleStartTime
    if not isTurningToAngle:
        turnToAngleStartTime = time.time()
        
    isTurningToAngle = True
    #startTime = time.time()
    maxTime = 10
    if angle < CheckAngle():
        left_motor.run(-speed)
        right_motor.run(speed)
        while angle < CheckAngle():
            if time.time() - turnToAngleStartTime > maxTime:
                TurnToAngleFailState(angle, speed, epsilon)
                return
            wait(20)

    elif angle > CheckAngle():
        left_motor.run(speed)
        right_motor.run(-speed)
        while angle > CheckAngle():
            if time.time() - turnToAngleStartTime > maxTime:
                TurnToAngleFailState(angle, speed, epsilon)
                return
            wait(20)
    
    left_motor.hold()
    right_motor.hold()
 
    wait(150)
    if angle > CheckAngle() + epsilon:
        TurnToAngle(angle, speed * 0.33, epsilon)
        return
    elif angle < CheckAngle() - epsilon:
        TurnToAngle(angle, speed * 0.33, epsilon)
        return
    isTurningToAngle = False


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
    robot.turn(40)
    robot.stop()
    ApproachLineStraight(-200)
    FollowLine(-450, -300)

# Flyt flaske
def Stage4():
    # Approach
    TurnToAngle(-180, 100, 1)
    AdjustGyro(2)
    #gyroSensor.reset_angle(-180)
    ResetGyro(-180)
    robot.straight(-225)
    robot.stop()
    TurnToAngle(-270, 150, 0.5)
    RunForkliftDown(45)
    robot.drive(-70, 0)
    while CheckDist() > 60:
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
    TurnToAngle(-180, 200, 4)
    RunForkliftUp(40)
    FollowLine(-450, -380)


# Venstresving over til vippen
def Stage5():
    global stage
    robot.straight(-300)
    robot.stop()
    TurnToAngle(-90, 200, 2)
    #gyroSensor.reset_angle(0)
    ResetGyro(0)
    FollowLine(-350, -300)

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
    TurnToAngle(-90, 200, 3)

    start_time = time.time()
    while True:
        color = CheckColor()
        if color >= (white + grey) / 2:
            right_motor.run(-300)
            left_motor.run(-400)
        elif color < (white + grey) / 2 and color > 15:
            right_motor.run(-400)
            left_motor.run(-300)
        if time.time() - start_time > 11:
            robot.stop()
            break
    TurnToAngle(CheckAngle() + 180, 200, 3)
    FollowLine(-400, -300)


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
    AdjustGyro(2.5)
    #TurnToAngle(90, 200, 0.5)
    robot.straight(-540)
    robot.stop()
    TurnToAngle(120 - 90, 100, 2)
    RunForkliftDown(45)
    
    robot.drive(-70, 0)
    while CheckDist() > 70:
        wait(50)
    robot.stop()

    RunForkliftUp(90)
    robot.straight(540)
    RunForkliftDown(65)
    robot.straight(200)
    robot.stop()
    TurnToAngle(240 - 90, 200, 3)
    RunForkliftUp(45)
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
    FollowLine(-400, -300)

# Rundt om flasken #1
def Stage11():
    robot.straight(-50)
    robot.stop()
    AdjustGyro(3)
    TurnToAngle(-50, 200, 3)
    ApproachLineStraight(-200)
    robot.straight(-200)
    robot.stop()
    TurnToAngle(-175, 200, 1)
    FollowLine(-250, -220)

# Zig-zag rundt om muren
def Stage12():
    robot.straight(-30)
    robot.stop()
    AdjustGyro(3.5)
    robot.straight(-380)
    robot.stop()
    TurnToAngle(40, 150, 1)
    robot.straight(-100)
    robot.stop()
    robot.drive(-170, -28)
    wait(3800)
    robot.stop()
    TurnToAngle(0, 200, 2)
    FollowLine(-300, -250)

    '''TurnToAngle(-260, 200, 3)
    robot.drive(-180, 15)
    while True:
        color = CheckColor()
        if color < white - 20:
            wait(300)
            robot.turn(-90)
            robot.stop()
            break
    FollowLine(-450, -300)'''

# Rundt om flasken #2
def Stage13():
    robot.straight(-100)
    robot.stop()
    TurnToAngle(-50, 200, 3)
    robot.drive(-180, 18)
    while True:
        color = CheckColor()
        if color < white - 20:
            break
    robot.stop()
    FollowLine(-300, -200)

# Landingsbane
def Stage14():
    TurnToAngle(160, 200, 2)
    robot.straight(-200)
    robot.stop()
    TurnToAngle(180, 200, 3)
    RunForkliftDown(45)
    start_time = time.time()
    while True:
        color = CheckColor()
        if color >= (white + grey) / 2:
            right_motor.run(-370)
            left_motor.run(-400)
        elif color < (white + grey) / 2 and color > 15:
            right_motor.run(-400)
            left_motor.run(-370)
        if CheckDist() < 1400 and time.time() - start_time > 4:
            robot.stop()
            break

    TurnToAngle(270, 100, 0)
    robot.straight(-100)
    robot.stop()
    TurnToAngle(180, 100, 1)

    RunForkliftUp(45)
    ev3.speaker.say('Wall-E')


StageControl()