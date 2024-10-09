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

def CheckAngle():
    return gyroSensor.angle()


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

    robot.stop()

def RunForkliftUp(power):
    small_motor.run_until_stalled(300, Stop.HOLD, power)

def RunForkliftDown(power):
    small_motor.run_until_stalled(-300, Stop.HOLD, power)


# Definér variabler
stage = 1
white = 80
grey = 40


# Funktion til at styre hvilket stadie på banen robotten er nået til
def StageControl():
    if stage == 1:
        Stage1()
    elif stage == 2:
        Stage2()
    elif stage == 3:
        Stage3()


# Del ét af brudt streg
def Stage1():
    global white
    global grey
    global stage
    gyroSensor.reset_angle(0)
    count = 0
    while count < 0.4:
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
    TurnToAngle(-90, 100, 2)
    stage += 1
    StageControl()


def Stage2():
    wait(100)
    FollowLine(-500, -530)


def Stage3():
    global white
    global grey
    wait(100)
    RunForkliftDown(45)
    while True:
        color = CheckColor()
        if color >= (white + grey) / 2:
            right_motor.run(-250)
            left_motor.run(-300)
        elif color < (white + grey) / 2 and color > 15:
            right_motor.run(-300)
            left_motor.run(-250)
        if CheckDist() < 80:
            robot.stop()
            break

StageControl()