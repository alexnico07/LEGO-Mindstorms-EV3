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

# Gear ratios = motor turns for every 1 turn of the joint.
# 3 means the motor spins 3 times for one turn of the joint (a reduction).
# If it turns out to be the other way round, change these to 1/3
BASE_RATIO = 3
SHOULDER_RATIO = 3
ELBOW_RATIO = 1

HOME_SPEED = 100   # deg/s at the motor shaft, kept slow on purpose
HOME_DUTY = 30     # % torque limit while homing, so a stall doesn't strain the gears
MOVE_SPEED = 150   # deg/s at the motor shaft for the test moves

# Test angles are ARM (joint) degrees, measured from the zero position
TEST_BASE = 45
TEST_SHOULDER = 30
TEST_ELBOW = 45

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


def go_to(motor, ratio, arm_angle):
    # Convert the joint angle to the motor angle, then move there
    motor.run_target(MOVE_SPEED, arm_angle * ratio, then=Stop.HOLD, wait=True)
    wait(300)


def arm_angle(motor, ratio):
    return round(motor.angle() / ratio)


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
go_to(base, BASE_RATIO, TEST_BASE)
go_to(base, BASE_RATIO, 0)

show("Shoulder")
go_to(shoulder, SHOULDER_RATIO, TEST_SHOULDER)
go_to(shoulder, SHOULDER_RATIO, 0)

show("Elbow")
go_to(elbow, ELBOW_RATIO, TEST_ELBOW)
go_to(elbow, ELBOW_RATIO, 0)

# 4. Report where each joint ended up in arm degrees (should all be close to 0)
show("A:{} B:{} C:{}".format(
    arm_angle(base, BASE_RATIO),
    arm_angle(shoulder, SHOULDER_RATIO),
    arm_angle(elbow, ELBOW_RATIO)))
ev3.speaker.beep()
wait(3000)