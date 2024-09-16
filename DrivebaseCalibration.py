#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)

# Afstandssensor og farvesensor initialiseres
colorSensor = ColorSensor(Port.S1)
distanceSensor = UltrasonicSensor(Port.S2)
gyroSensor = GyroSensor(Port.S3, Direction.CLOCKWISE)
touchSensor = TouchSensor(Port.S4)

# Funktioner som tjekker farvereflektion, afstand og rumlig orientering initialiseres
def CheckColor():
    return colorSensor.reflection()

def CheckDist():
    return distanceSensor.distance()

def CheckAngle():
    return gyroSensor.angle()

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=70, axle_track=136) #todo: calibrate

def TurnToAngle(angle, speed):
    left_motor.run(speed)
    right_motor.run(-speed)
    if angle < gyroSensor.angle():
        while angle < gyroSensor.angle()
        pass
    elif angle > gyroSensor.angle():
        while angle > gyroSensor.angle()
        pass
    
    left_motor.hold()
    right_motor.hold()
 
    if angle > gyroSensor.angle() :
        TurnToAngle(angle, speed * 0.5)
    elif angle < gyroSensor.angle():
        TurnToAngle(angle, speed * 0.5)

# robot.straight(-210) #længen af et A5 papir


gyroSensor.reset_angle(0)


TurnToAngle(360, 100)

wait(1000)
ev3.speaker.say("Gyroscope angle is: "+ str(gyroSensor.angle()))

"""
robot.reset()
robot.drive(0, 20)
wait(5000)
robot.stop()
wait(1000)
robot.turn(-robot.angle())
"""






    # step 1: find edge to white
    # step 2: cache current gyroscope angle
    # step 3: turn very slowly in the grey
    # step 4: when you hit white, cache current gyroscope angle and stop
        # debug step, maybe: repeat a couple of times
    # step 5: calculate distance between the two points, as a triangle. Det giver hypotenusen på en retvinklet trekant,
    # dette er trekantA

    # der består af: b: banens pre-definerede bredde, c: den distance vi lige har udregnet og en ukendt a.
    # denne retvinklede trekant er trekantA.

    # vi kan også finde vinklen i bundene af sidebenene, triGyroCalculatedAngle = (180 - triGyroMeasuredAngle)/2
    # den ukendte side, a findes med: sqrt(c²-b²)

    # derefter findes vinklen i trekantA, A_alpha (modstående side A)
    # det gøres med A_alpha = arcsin(a / c)

    # nu har kan vi lægge 90, vinkal A_alpha og sidebensvinklen sammen og trække det fra 180 - så har vi vinkel b_alpha
    # for at finde side a på trekantB, er formlen: a = c * cos(a)
    # hvis jeg så tager halvdelen af grå linjes bredde, og trækker side a fra dette, får jeg side b i trekantD

    # hvis jeg trækker vinkel b_alpha fra det ligebenede trekants gyromåling, har jeg vinkel d_alpha
    # dette er vinkel v, og er det jeg skal dreje
    # For at finde hvor langt jeg skal køre skal jeg udregne hypotenusen i trekantD
    # først finder jeg siden a i trekantD, da denne skal bruges til at finde hypotenusen:
    # Det finder jeg med vinklen D_alpha, jeg kan udregne som D_alpha = 180 - triGyroMeasuredAngle - B_beta
    # Ud fra det kan jeg finde D_beta ved hjælp af vinkelsummen. D_beta = 90 - D_alpha
    # så findes siden b i trekantD med: triDSideB = TriDSideA * tan(TriD_beta)
    # dist = sqrt(D_a² + D_b²)
    # så kører robotten dette

    # nem trig lookup: https://www.omnicalculator.com/math/right-triangle-side-angle
            




