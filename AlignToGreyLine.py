#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

class RightAngleTri(Enum):
    sideA = 0.0
    sideB = 0.0
    sideC = 0.0
    alpha = 0.0
    beta = 0.0

class GyroTriangle:
    def __init__(self, measuredAngle, legLength):
        self.measuredAngle = radians(measuredAngle)
        self.legLength = legLength
        self.grundlinje = 2 * legLength * sin(measuredAngle)
        self.oppositeAngles = (180 - measuredAngle)*0.5



# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=124) #todo: calibrate

# Gyroskop og farvesensor
Farvesensor = ColorSensor(Port.S3)
gyroSensor = GyroSensor(Port.S3)

def CheckColor():
    return Farvesensor.reflection()

def CheckDist():
    return Afstandssensor.distance()

def CheckGyro():
    return gyroSensor.angle()

GREY_LINE_WIDTH = 50
DIST_TURN_AXIS_TO_COLOR_CHECKER = 65 # millimeter

axisGyroEntry = 0
axisGyroExit = 0

def CalculateMiddleOfLineFromInside(gyroTri):
    # initialize trekant klassserne
    triA = RightAngleTri()
    triB = RightAngleTri()
    triC = RightAngleTri()

    # trekant a defineres
    triA.sideB = GREY_LINE_WIDTH #side b er blot bredden af linjen
    triA.sideC = gyroTri.grundlinje # hypotenusen er grundlinjen på den målte ligebenede trekant
    triA.sideA = sqrt(pow(triA.sideC, 2) - pow(triA.sideB, 2)) # side a udregnes herfra

    triA.alpha = asin(triA.sideA / triA.sideC) #jeg finder vinklen alpha på trekant a
    
    triB.alpha = 90 - (degrees(triA.alpha) + gyroTri.oppositeAngles) # ud fra dette udregne trekant b's vinkel beta
    triB.beta = 90 - triB.alpha #med dette kan modstående vinkel også findes
    outputAngle = gyroTri.measuredAngle - triB.alpha #vinkel alpha kan trækkes fra den målte vinkel, og hermed ved vi hvor langt vi skal dreje.

    triB.sideC = gyroTri.legLength #dette er længden fra omdrejningspunktet til farvelæseren
    triB.sideA = triB.sideC * cos(radians(triB.alpha)) #side a findes i trekant b, da denne skal bruges til at finde side b i trekant D


    triC.sideB = GREY_LINE_WIDTH * 0.5 - triB.sideA
    triC.alpha = 180 - (gyrotri.measuredAngle + triB.beta)
    triC.sideA = triD.sideB * tan(radians(triD.alpha))

    triC.sideC = sqrt(pow(triD.sideA, 2) + pow(triD.sideB, 2))
    outputDistance = triD.sideC

    return (outputDistance, outputAngle)

def CalclulateMiddleOfLineFromOutside(gyroTri):
     # initialize trekant klassserne
    triA = RightAngleTri()
    triB = RightAngleTri()
    triC = RightAngleTri()

    # trekant a defineres
    triA.sideB = GREY_LINE_WIDTH #side b er blot bredden af linjen
    triA.sideC = gyroTri.grundlinje # hypotenusen er grundlinjen på den målte ligebenede trekant
    triA.sideA = sqrt(pow(triA.sideC, 2) - pow(triA.sideB, 2)) # side a udregnes herfra

    triA.alpha = asin(triA.sideA / triA.sideC) #jeg finder vinklen alpha på trekant a
    
    triB.beta = 90 - (degrees(triA.alpha) + gyroTri.oppositeAngles) # ud fra dette udregne trekant b's vinkel beta
    triB.alpha = 90 - triB.beta #med dette kan modstående vinkel også findes
    outputAngle = gyroTri.measuredAngle + triB.alpha #vinkel alpha kan trækkes fra den målte vinkel, og hermed ved vi hvor langt vi skal dreje.

    triB.sideC = gyroTri.legLength #dette er længden fra omdrejningspunktet til farvelæseren
    triB.sideA = triB.sideC * cos(radians(triB.alpha)) #side a findes i trekant b, da denne skal bruges til at finde side b i trekant D


    triC.sideB = GREY_LINE_WIDTH * 0.5 + triB.sideA
    triC.beta = 90 - (gyrotri.measuredAngle + triB.alpha)
    triC.sideA = triD.sideB * tan(radians(triD.alpha))

    triC.sideC = sqrt(pow(triD.sideA, 2) + pow(triD.sideB, 2))
    outputDistance = triD.sideC

    return (outputDistance, outputAngle)


def AlignToGreyLine(isOnRightSide, isOutsideLine):
    if isOnrightSide:
        robot.turn(20) #creates buffer to grey line
        wait(100)
        gyroSensor = GyroSensor(Port.S3, Direction.COUNTERCLOCKWISE)
        instructions = AnalyzeGreyLine(100, isOutsideLine)
    else:
        robot.turn(-20)
        wait(100)
        gyroSensor = GyroSensor(Port.S3, Direction.CLOCKWISE)
        instructions = AnalyzeGreyLine(-100, isOutsideLine)
    
    robot.straight(instructions[0])
    #robot.turn(instructions[1])
    #TurnAngle(instructions[1])
    if isOnRightSide:
        gyroSensor = GyroSensor(Port.S3, Direction.CLOCKWISE)
        TurnAngle(instructions[1], 100)
    else:
        gyroSensor = GyroSensor(Port.S3, Direction.COUNTERCLOCKWISE)
        TurnAngle(instructions[1], -100)
    

    

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



def AnalyzeGreyLine(turnSpeed, isOutsideLine):
    global white
    robot.stop()
    # reset gyroscop, eller cache nuværende værdi

    left_motor.run(turnSpeed)
    right_motor.run(-turnSpeed)

    while CheckColor() >= white: 
        pass

    gyroSensor.reset_angle(0)

    while CheckColor() <= white: 
        pass

    left_motor.stop()
    right_motor.stop()

    angleGyroExit = CheckGyro()
    if isOutsideLine:
        instructions = CalculateMiddleOfLineFromOutside(GyroTriangle(angleGyroExit - angleGyroEntry, DIST_TURN_AXIS_TO_COLOR_CHECKER))
    else:
        instructions = CalculateMiddleOfLineFromInside(GyroTriangle(gyroSensor.angle(), DIST_TURN_AXIS_TO_COLOR_CHECKER))
    return instructions


AlignToGreyLine(True, True)



    





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
            




