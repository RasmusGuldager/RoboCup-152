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
forklift_motor_left = Motor(Port.D)
forklift_motor_right = Motor(Port.C)
touchSensor = TouchSensor(Port.S3)


FORKLIFT_DUTY_POWER = 100



def RunForkliftUp():
    #forklift_motor.hold()
    
    ev3.speaker.say("Raising forklift")
    forklift_motor_left.stop()
    forklift_motor_right.stop()

    forklift_motor_left.run(-100)
    forklift_motor_right.run(100)

    #forklift_motor_left.run_until_stalled(-200, Stop.HOLD, 100)
    #forklift_motor_right.run_until_stalled(200, Stop.HOLD, 100)
    
    

def RunForkliftDown():
    #forklift_motor.hold()
    
    ev3.speaker.say("Lowering forklift")
    forklift_motor_left.stop()
    forklift_motor_right.stop()

    forklift_motor_left.run(100)
    forklift_motor_right.run(-100)

    #forklift_motor_left.run_until_stalled(100, Stop.HOLD, 75)
    #forklift_motor_right.run_until_stalled(-100, Stop.HOLD, 75)



RunForkliftUp()

isGoingUp = True

while (True):
    if(touchSensor.pressed()):
        if(isGoingUp):
            RunForkliftDown()
            isGoingUp = False
        else:
            RunForkliftUp()
            isGoingUp = True


