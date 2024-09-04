#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile



def color_calibration()
    if (touch_sensor.pressed()):
        BLACK = line_sensor.reflection()
        if BLACK == 0:
            return 0
            
    if (touch_sensor.pressed()):
        WHITE = line_sensor.reflection()
        if WHITE == 0:
            return 0
        else:
            return 1

def follow_line_motors()

    # sort og hvis værdier til kalibrering 
    BLACK = 3
    WHITE = 62
    # denne værdi er hvornår den er hvid eller sort
    threshold = (BLACK+WHITE) / 2
    # hastighed på Motor
    DRIVE_SPEED = 100
    # Hvor meget kræft den skal dreje med
    PROPORTIONAL_GAIN=1.4

def main()
    while menu==1:
        if (touch_sensor.pressed()):
            status = color_calibration()


    while status==1:
        # hvor meget den er væk fra stregen
        error = line_sensor.reflection()-threshold
        # hvor meget kræft den skal dreje med bliver større jo længere væk den er fra starten
        turn_rate = PROPORTIONAL_GAIN * error
        # drejer motor
        left_motor(int(DRIVE_SPEED+turn_rate))
        right_motor(int(DRIVE_SPEED-turn_rate))
        wait(10)

if __name__ == "__main__":
    print("start")
    sound = SoundFile()
    touch_sensor = TouchSensor(Port.S1)
    # venstre motor
    left_motor = Motor(Port.A,Direction.ClOCKWISE)
    # højre motor
    right_motor = Motor(Port.B,Direction.ClOCKWISE)
    # farve sensor
    line_sensor = ColorSensor(Port.S3)
    status = 0
    menu = 1
    main()
