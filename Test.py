#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Direction, Stop, Button
from pybricks.tools import wait

ev3 = EV3Brick()

# ---------------------------------------------------------------
# SETTINGS TO TUNE ON THE FIRST RUN
# ---------------------------------------------------------------
# Homing always drives at a NEGATIVE speed, into the limit stop.
# So each Direction below must be chosen so that "negative" = towards the stop.
# If a joint drives AWAY from its stop during homing, flip its Direction
# (CLOCKWISE <-> COUNTERCLOCKWISE). After homing, positive angles = away from the stop.
BASE_DIR = Direction.CLOCKWISE
SHOULDER_DIR = Direction.CLOCKWISE
ELBOW_DIR = Direction.CLOCKWISE  # no stop on the elbow, so this just sets which way is positive

HOME_SPEED = 100   # deg/s at the motor shaft, kept slow on purpose
HOME_DUTY = 30     # % torque limit while homing, so a stall doesn't strain the gears
MOVE_SPEED = 100   # deg/s at the motor shaft for the test moves

# Angles below are MOTOR shaft degrees (not arm degrees), because the gearing
# between each motor and its joint isn't entered here yet.
TEST_BASE = 90
TEST_SHOULDER = 60
TEST_ELBOW = 60

# ---------------------------------------------------------------
# MOTORS (base = A, shoulder = B, elbow = C)
# ---------------------------------------------------------------
base = Motor(Port.A, BASE_DIR)
shoulder = Motor(Port.B, SHOULDER_DIR)
elbow = Motor(Port.C, ELBOW_DIR)


def home_against_stop(motor):
    # Drive slowly into the stop until the motor stalls, then call that angle 0
    motor.run_until_stalled(-HOME_SPEED, then=Stop.COAST, duty_limit=HOME_DUTY)
    motor.reset_angle(0)


def show(text):
    ev3.screen.clear()
    ev3.screen.print(text)
    print(text)


def go_to(motor, angle):
    motor.run_target(MOVE_SPEED, angle, then=Stop.HOLD, wait=True)
    wait(300)


# ---------------------------------------------------------------
# START: wait for the centre button so nothing moves on upload
# ---------------------------------------------------------------
show("Centre button\nto start")
while Button.CENTER not in ev3.buttons.pressed():
    wait(10)
wait(500)
ev3.speaker.beep()

# 1. Home the two joints that have a limit stop
show("Homing shoulder")
home_against_stop(shoulder)

show("Homing base")
home_against_stop(base)

# 2. Elbow has no stop: whatever pose it is in right now becomes 0
elbow.reset_angle(0)

show("Homed. Moving...")
wait(1000)

# 3. Small test moves, one joint at a time, returning to 0 after each
show("Base")
go_to(base, TEST_BASE)
go_to(base, 0)

show("Shoulder")
go_to(shoulder, TEST_SHOULDER)
go_to(shoulder, 0)

show("Elbow")
go_to(elbow, TEST_ELBOW)
go_to(elbow, 0)

# 4. Report where each joint ended up (should all be close to 0)
show("A:{} B:{} C:{}".format(base.angle(), shoulder.angle(), elbow.angle()))
ev3.speaker.beep()
wait(3000)