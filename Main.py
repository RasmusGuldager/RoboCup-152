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

# Definér konstanter
grey = 43
black = 7
white = 74

# hastighed på Motor
DRIVE_SPEED = 400

# PID konstaten til at kalibrere
PROPORTIONAL_GAIN= 4.2
INTERGRAL_GAIN = 0.003
DERIVATIVE_GAIN = 0.02
# Limit for integral for at undga winup
INTEGRAL_MAX = 100
INTEGRAL_MIN = -100

# Definér variabler
colorStage = 1
stage = 0
# Reset Gyro sensor vinkel til 0
gyroSensor.reset_angle(0)


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
    global grey
    global colorStage
    global stage
    while True:
        if touchSensor.pressed() and colorStage == 1:
            white = CheckColor()
            ev3.speaker.say("White " + str(white))
            colorStage += 1
        elif touchSensor.pressed() and colorStage == 2:
            black = CheckColor()
            ev3.speaker.say("Black " + str(black))
            stage += 1
        elif touchSensor.pressed() and colorStage == 3:
            grey = CheckColor()
            ev3.speaker.say("Grey " + str(grey))
            stage += 1
            break

    wait(5000)
    StageControl()

def FollowLine():
    global white
    global black
    global stage
    global right_motor
    global left_motor
    counter = 0
    integral = 0
    derivative = 0
    last_error = 0
    threshold = (grey+white) / 2
    while True:
        color = line_sensor.reflection()

        error = color - threshold
        integral = integral + error

        if integral > INTEGRAL_MAX:
            integral = INTEGRAL_MAX
        elif integral < INTEGRAL_MIN:
            integral = INTEGRAL_MIN

        derivative = error - last_error


        turn_rate = PROPORTIONAL_GAIN * error + INTERGRAL_GAIN * integral + DERIVATIVE_GAIN * derivative

        motor_speedlog.log(error, integral, derivative, turn_rate)

        left_motor.run(DRIVE_SPEED-turn_rate)
        right_motor.run(DRIVE_SPEED+turn_rate)

        last_error = error
        elif color < black + 10: 
            break
    robot.stop()
    stage += 1
    StageControl()
            
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
    FollowLine()

# Del to af brudt streg
def Stage2():
    robot.turn(-30)
    robot.straight(-400)
    robot.turn(30)
    robot.stop()
    FollowLine()

# 180 grader sving
def Stage3():
    robot.turn(30)
    robot.straight(-400)
    robot.turn(-30)
    robot.stop()
    FollowLine()

# Flyt flaske
def Stage4():
    pass

# Venstresving over til vippen
def Stage5():
    pass

# Vippen
def Stage6():
    pass

# Parallelle streger
def Stage7():
    pass

# Venstresving over til dartskive
def Stage8():
    pass

# Dartskive
def Stage9():
    pass

# Fra dartskive over til rundt om flasken
def Stage10():
    pass

# Rundt om flasken #1
def Stage11():
    pass

# Zig-zag rundt om muren
def Stage12():
    pass

# Rundt om flasken #2
def Stage13():
    pass

# Landingsbane
def Stage14():
    pass

StageControl()

            