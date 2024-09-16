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
small_motor = Motor(Port.C)
touchSensor = TouchSensor(Port.S4)


FORKLIFT_DUTY_POWER = 100



def RunForkliftUp():
    #forklift_motor.hold()
    
    ev3.speaker.say("Raising forklift")
    small_motor.stop()

    #forklift_motor.run(200)
    small_motor.run(500)

    #forklift_motor_left.run_until_stalled(-200, Stop.HOLD, 100)
    #forklift_motor_right.run_until_stalled(200, Stop.HOLD, 100)
    
    

def RunForkliftDown():
    #forklift_motor.hold()
    
    ev3.speaker.say("Lowering forklift")
    small_motor.stop()

    #forklift_motor.run(-200)
    small_motor.run_until_stalled(-300, Stop.HOLD, 60)

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


