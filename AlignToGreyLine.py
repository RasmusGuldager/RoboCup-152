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

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=124)

# Afstandssensor og farvesensor
Farvesensor = ColorSensor(Port.S3)

def CheckColor():
    return Farvesensor.reflection()

def CheckDist():
    return Afstandssensor.distance()

# method to check current angle here


watch = StopWatch() #definer et stopur
prevColor = checkColor() #en cached farveværdi, så de kan sammenlignes

GREY_LINE_WIDTH = 50
DIST_TURN_AXIS_TO_COLOR_CHECKER = 65



watch.resume() #start uret
cascadeItarator = 0 #Sæt en iterator


while True:
    # step 1: find edge to white
    # step 2: cache current gyroscope angle
    # step 3: turn very slowly in the grey
    # step 4: when you hit white, cache current gyroscope angle and stop
        # debug step, maybe: repeat a couple of times
    # step 5: calculate distance between the two points, as a triangle. Det giver hypotenusen på en retvinklet trekant,
    # der består af: b: banens pre-definerede bredde, c: den distance vi lige har udregnet og en ukendt a.
    # a findes, og så kan vinklen mellem c og b findes.
    color = checkColor()
    if color > 50:
        if prevColor <= 50: #hvis den lige har skiftet farve, så reset uret
            watch.reset()
        prevColor = color #farve sættes til nuværende farve for næste loop

        TurnLeft()
    else:
        if prevColor > 50: #hvis den lige har skiftet farve, så reset uret
            watch.reset()
        prevColor = color #farve sættes til nuværende farve for næste loop

        TurnRight()


# step 1: find


            




