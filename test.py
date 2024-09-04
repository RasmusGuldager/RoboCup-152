#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Initialize the EV3 Brick.
ev3 = EV3Brick()

def follow_line_motors()
    # venstre motor
    left_motor = Motor(Port.A,Direction.ClOCKWISE)
    # højre motor
    right_motor = Motor(Port.B,Direction.ClOCKWISE)
    # farve sensor
    line_sensor = ColorSensor(Port.S3)

    # sort og hvis værdier til calbriing 
    BLACK = 3
    WHITE = 62
    # denne værdi er hvornår den er hvid eller sort
    threshold = (BLACK+WHITE) / 2
    # hastighed på Motor
    DRIVE_SPEED = 100
    # Hvor meget kræft den skal dreje med
    PROPORTIONAL_GAIN=1.4

while True:
    # hvor meget den er væk fra stregen
    error = line_sensor.reflection()-threshold
    # hvor meget kræft den skal dreje med bliver større jo længere væk den er fra starten
    turn_rate = PROPORTIONAL_GAIN * error
    # drejer motor
    left_motor(int(DRIVE_SPEED+turn_rate))
    right_motor(int(DRIVE_SPEED-turn_rate))
    wait(10)
