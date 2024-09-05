#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# intialisere Motorere
left_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
# intialisere Farve sensor
line_sensor = ColorSensor(Port.S2)
# laver en log af værdier
motor_speedlog = DataLog('error', 'integral', 'derivative', 'turn_rate', name='motor_speedlog')
# sort og hvis værdier til kalibrering 
GREY = 43
BLACK = 7
WHITE = 74
# denne værdi er refleksion imellem sort og hvid
threshold = (GREY+WHITE) / 2
# hastighed på Motor
DRIVE_SPEED = 400
# PID konstaten til at kalibrere
PROPORTIONAL_GAIN= 6
INTERGRAL_GAIN = 0.04
DERIVATIVE_GAIN = 0.02
# PID variablerF
integral = 0
derivative = 0
last_error = 0

def main():
    global integral, last_error  # Declare global to modify them inside the function






main()