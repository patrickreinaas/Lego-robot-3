#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math # For pi and trigonometric functions
import random

#Initialize the EV3 brick
ev3 = EV3Brick()
ev3.speaker.beep()

# Sets up motors and sensors
right_motor = Motor(Port.A)
left_motor = Motor(Port.B)
ts = TouchSensor(Port.S1)
uss = UltrasonicSensor(Port.S2)
cs = ColorSensor(Port.S4)
timer = StopWatch()

# Robot's physical variables
wheel_diameter = 55.5 # mm
axle_track = 125 # mm

#Initialize the drivebase
db = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=125)

# State management variables
is_running = True
is_on = False


# Entertainment acts

# Underholdingsnummere: Spin 360 degrees, read a poem, sing the melody to a song, do a dance.
def act_spin():
    ev3.speaker.say("360")
    db.turn(450)

#Reads a simple poem
def act_poem():
    lines = [ "Roses are red",
            "Circuits are bright",
            "Following the black line"]
    for line in lines:
        ev3.speaker.say(line)
        wait(140)


#Plays the melody to Dovregubbens Hall
def act_sing():
    ev3.speaker.say("Dovregubbens Hall")
    wait(50)
    melody = [494, 554, 587, 659, 740, 587, 659, 740,
        784, 659, 740, 784, 880, 740, 784, 880]
    base = 190          # start duration (ms) for every tone
    faster_step = 30    # how much we shorten the tones every repeat
    repeats = 3   #repeats melody 3 times, each faster than the last one
    
    for r in range(repeats):
        dur = max(80, base - r * faster_step)
        for f in melody:
            ev3.speaker.beep(frequency = f, duration = dur)
            wait(20)
    
#Makes the robot do a short dance
def act_dance():
    ev3.speaker.say("Dance")
    for _ in range(2):
        db.turn(60)
        db.turn(-60)
        ev3.speaker.beep(700, 120)
        wait(100)

#List with the different acts
Underholding = [act_spin, act_poem, act_sing, act_dance]

#Makes the robot do a random act from the list
def do_random_act():
    db.stop()
    act = random.choice(Underholding)
    act()

    wait(200)




    

class EntertainmentRobot:
    
    # Automatically invoked method for setting instance variables (python version of a constructor basically)
    def __init__(self, drive_speed, turn_rate):
        self.drive_speed = drive_speed # mm/s
        self.turn_rate = turn_rate # degrees/s
        self.is_done = False # Bool to check if the program should stop


    # Follows line based on given black/white surface reflection values and a factor 
    def follow_line(self, color_black, color_white, turn_factor):

        # Tries to find line again if robots starts away from line
        degrees_turned = 0
        while ((cs.reflection() > 30) or (degrees_turned < 360)):
            db.drive(20, 45)
            degrees_turned += 10

        timer.reset() # Resets time counter

        # Sets the margin to calculate turn rate from
        threshold = (color_black + color_white) / 2

        while (timer.time() < 10000): # Runs while timer is less than 10 seconds

            # Calculates turn rate based on surface reflection 
            deviation = cs.reflection() - threshold

            # Sets turn rate
            self.turn_rate = deviation * turn_factor

            db.drive(self.drive_speed, self.turn_rate)

            # Runs while an obstacle has been sensed but not gotten too close yet
            while (uss.distance() <= 200):

                # Slows down in front of obstacle
                db.drive(int(self.drive_speed/2), self.turn_rate)
                
                if (uss.distance() <= 100): # If distance to obstacle is 10 cm or less
                    db.stop()
                    ev3.speaker.play_file(SoundFile.CHEERING)
                    is_done = True
                    finish_program()
                    return

        if (timer.time() >= 10000): # Does a random act if 10 seconds have passed
            do_random_act()
            return

# Creates an instance of the entertainment robot
robot = EntertainmentRobot(drive_speed=90, turn_rate=0)

# Runs as long as robot is not done
def robot_loop():
    if (robot.is_done):
        finish_program()
        return

    # Follows line
    robot.follow_line(color_black=0, color_white=50, turn_factor=2.0)

# Ends program
def finish_program():
    global is_running
    global is_on

    db.stop() # Extra stop to the drivebase just in case

    # Sets state management variables to false to ensure ending the program loop
    is_on = False
    is_running = False

    return

# Main program loop
while (is_running):
    if ((ts.pressed()) and (is_on == False)):
        is_on = True
    
    while (is_on):
        robot_loop()