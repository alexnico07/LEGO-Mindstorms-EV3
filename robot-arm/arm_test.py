#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, InfraredSensor
from pybricks.parameters import Port, Direction, Stop, Button
from pybricks.tools import wait, StopWatch

ev3 = EV3Brick()

# ---------------------------------------------------------------
# WHAT TO RUN THIS TIME
# ---------------------------------------------------------------
# Pick which joints get homed when the touch sensor is pressed. Turn one off
# if it's already sitting at its zero and you only want to home the others.
# (Remote jogging and the demo assume all three have been homed.)
HOME_BASE = True
HOME_SHOULDER = True
HOME_ELBOW = True

# ---------------------------------------------------------------
# SETTINGS TO TUNE ON THE FIRST RUN
# ---------------------------------------------------------------
# Homing always drives at a NEGATIVE speed, into the limit stop.
# So each Direction below must be chosen so that "negative" = towards the stop.
# If a joint drives AWAY from its stop during homing, flip its Direction
# (CLOCKWISE <-> COUNTERCLOCKWISE). After homing, positive angles = away from the stop.
BASE_DIR = Direction.CLOCKWISE
SHOULDER_DIR = Direction.CLOCKWISE
# Elbow homes into its stop by turning the motor CW (looking down at it), and homing
# is a NEGATIVE speed, so negative = CW, i.e. positive = CCW. If it drives away from
# the stop instead, flip this to CLOCKWISE.
ELBOW_DIR = Direction.COUNTERCLOCKWISE

# Gear ratios = motor turns for every 1 turn of the joint.
# 3 means the motor spins 3 times for one turn of the joint (a reduction).
# If it turns out to be the other way round, change these to 1/3
BASE_RATIO = 3
SHOULDER_RATIO = 3
# Elbow was rebuilt with a 20:12 gear pair: motor drives the 12-tooth gear, the
# 20-tooth gear is on the arm, so the motor turns 20/12 (about 1.67) times per
# joint turn. If it is the other way round, use 12 / 20 instead.
ELBOW_RATIO = 20 / 12

# Speeds are in deg/s at the JOINT; the code multiplies by each joint's gear
# ratio to get the motor-shaft speed. (20 and 50 match the old base/shoulder
# shaft speeds of 60 and 150 at 3:1.)
HOME_SPEED = 20   # deg/s at the joint, kept slow on purpose
MOVE_SPEED = 50   # deg/s at the joint for the test moves

# The elbow is jerky when slow. The speed sweep (done at 1:1, so these were
# MOTOR deg/s) found 200 and 300 smoothest, so it gets its own speeds, still in
# deg/s at the joint. With the 20:12 ratio, 120 joint deg/s = 200 motor deg/s
# and 60 joint deg/s = 100 motor deg/s, so the motor runs at the speeds tested.
ELBOW_MOVE_SPEED = 120
ELBOW_HOME_SPEED = 60  # untested into the stop: hits harder than the others, raise gradually

# Homing duty is tuned per joint — keep these separate, don't let one inherit the other.
BASE_HOME_DUTY = 15      # % torque limit while homing the base
SHOULDER_HOME_DUTY = 15  # % torque limit while homing the shoulder (confirmed working)
ELBOW_HOME_DUTY = 20     # % torque limit while homing the elbow (needs >20% to move at all)

# Maximum ARM (joint) angles, measured from each joint's homed zero.
# Hard limits: go_to() clamps every commanded target to these, so the arm
# never goes past them. No far-limit stall testing is done any more.
BASE_MAX_ANGLE = 200
SHOULDER_MAX_ANGLE = 145
# Elbow has no confirmed limit yet, so go_to() does not clamp it. Remote jogging
# uses the provisional ELBOW_JOG_MAX below instead.

# Test angles are ARM (joint) degrees, measured from the zero position
TEST_BASE = 45
TEST_SHOULDER = 30
TEST_ELBOW = 45

# After homing, each joint turns back (away from the stop) by this many ARM degrees.
# Clamped to the max angles above.
BASE_TURN_BACK_AFTER_HOME = 120
SHOULDER_TURN_BACK_AFTER_HOME = 90
ELBOW_TURN_BACK_AFTER_HOME = 135

# The pose homing leaves the arm in (base, shoulder, elbow), in ARM degrees
READY_POSE = (BASE_TURN_BACK_AFTER_HOME,
              SHOULDER_TURN_BACK_AFTER_HOME,
              ELBOW_TURN_BACK_AFTER_HOME)

# ---------------------------------------------------------------
# SENSORS AND REMOTE
# ---------------------------------------------------------------
touch = TouchSensor(Port.S1)   # press = run the homing sequence
ir = InfraredSensor(Port.S4)   # reads the EV3 infrared remote

# The red switch on the remote selects channel 1 (top) to 4 (bottom). The LEFT
# pair of buttons always moves the base. What the RIGHT pair moves depends on the
# switch: (channel, joint index) with 1 = shoulder and 2 = elbow. Channel 4 is unused.
REMOTE_CHANNELS = [(1, 1), (2, 2), (3, 2)]

# While jogging, stop this many ARM degrees short of each end of a joint's range
# (0 and the max angle), so the remote never drives a joint into its hard stop.
REMOTE_LIMIT_MARGIN = 5

# Jog speeds while a remote button is held, in deg/s at the JOINT.
BASE_JOG_SPEED = 50
SHOULDER_JOG_SPEED = 50
ELBOW_JOG_SPEED = 120  # = 200 motor deg/s, the smoothest elbow speed, see above

# Elbow upper limit for remote jogging, in arm degrees. None = no limit.
# PROVISIONAL: 180 is not a measured limit. It is the largest elbow angle already
# commanded without trouble on the old elbow build. Replace it with the real
# maximum once the gripper motor is fitted and the elbow's range is known.
ELBOW_JOG_MAX = 180

# ---------------------------------------------------------------
# MOTORS (base = A, shoulder = B, elbow = C)
# ---------------------------------------------------------------
base = Motor(Port.A, BASE_DIR)
shoulder = Motor(Port.B, SHOULDER_DIR)
elbow = Motor(Port.C, ELBOW_DIR)


def home_against_stop(motor, duty, ratio, speed=HOME_SPEED):
    motor.run_until_stalled(round(-speed * ratio), then=Stop.COAST, duty_limit=duty)
    print("angle at stall:", motor.angle())
    wait(500)
    print("angle after settling:", motor.angle())
    motor.reset_angle(0)


def show(text):
    ev3.screen.clear()
    ev3.screen.print(text)
    print(text)


def go_to(motor, ratio, target, max_angle=None, speed=MOVE_SPEED):
    # Joints with a max angle (base, shoulder) also have a hard stop at 0,
    # so clamp to 0..max_angle. The elbow passes no max, so it is not clamped.
    # Convert the joint angle to the motor angle, then move there.
    if max_angle is not None:
        if target > max_angle:
            print("target {} over max {}, clamping".format(target, max_angle))
            target = max_angle
        elif target < 0:
            print("target {} below 0, clamping".format(target))
            target = 0
    motor.run_target(round(speed * ratio), round(target * ratio), then=Stop.HOLD, wait=True)
    wait(300)


def go_to_logged(motor, ratio, target, speed, ms=4000):
    # Same move as go_to(), but non-blocking, printing time (ms), motor angle
    # and motor speed every 250 ms so a stuck move shows up in the log.
    motor.run_target(round(speed * ratio), round(target * ratio), then=Stop.HOLD, wait=False)
    sw = StopWatch()
    while sw.time() < ms:
        print(sw.time(), motor.angle(), motor.speed())
        wait(250)


def arm_angle(motor, ratio):
    return round(motor.angle() / ratio)


def home_base():
    show("Homing base")
    home_against_stop(base, BASE_HOME_DUTY, BASE_RATIO)
    print("base angle after homing:", base.angle())

    show("Base back {} deg".format(BASE_TURN_BACK_AFTER_HOME))
    go_to(base, BASE_RATIO, BASE_TURN_BACK_AFTER_HOME, BASE_MAX_ANGLE)
    print("base angle after turn-back:", base.angle())


def home_shoulder():
    show("Homing shoulder")
    home_against_stop(shoulder, SHOULDER_HOME_DUTY, SHOULDER_RATIO)
    print("shoulder angle after homing:", shoulder.angle())

    show("Shoulder back {} deg".format(SHOULDER_TURN_BACK_AFTER_HOME))
    go_to(shoulder, SHOULDER_RATIO, SHOULDER_TURN_BACK_AFTER_HOME, SHOULDER_MAX_ANGLE)
    print("shoulder angle after turn-back:", shoulder.angle())


def home_elbow():
    show("Homing elbow")
    home_against_stop(elbow, ELBOW_HOME_DUTY, ELBOW_RATIO, ELBOW_HOME_SPEED)
    print("elbow angle after homing:", elbow.angle())

    show("Elbow back {} deg".format(ELBOW_TURN_BACK_AFTER_HOME))
    go_to(elbow, ELBOW_RATIO, ELBOW_TURN_BACK_AFTER_HOME, speed=ELBOW_MOVE_SPEED)
    print("elbow angle after turn-back:", elbow.angle())


def home_selected():
    # Order matters less here than with the arm hanging under load, but
    # shoulder-then-base matches the original full sequence.
    if HOME_SHOULDER:
        home_shoulder()
    if HOME_BASE:
        home_base()
    if HOME_ELBOW:
        home_elbow()


def go_pose(b, s, e):
    # Move to a pose given in ARM degrees, one joint at a time:
    # shoulder, then elbow, then base. Same clamps and speeds as go_to().
    go_to(shoulder, SHOULDER_RATIO, s, SHOULDER_MAX_ANGLE)
    go_to(elbow, ELBOW_RATIO, e, speed=ELBOW_MOVE_SPEED)
    go_to(base, BASE_RATIO, b, BASE_MAX_ANGLE)


def demo():
    # Starts from the ready pose (the three turn-back angles) and returns
    # there at the end. All angles are ARM degrees and must stay inside each
    # joint's range (base 0-200, shoulder 0-145, elbow >= 0).
    # The angles are a first guess: check clearance before trusting them.
    ready = READY_POSE

    # 0. The arm may have been jogged somewhere else, so go to the ready pose first
    show("Demo: ready pose")
    go_pose(*ready)

    # 1. Base sweep: one way, the other way, back to the ready angle
    show("Demo: base")
    go_to(base, BASE_RATIO, 60, BASE_MAX_ANGLE)
    go_to(base, BASE_RATIO, 180, BASE_MAX_ANGLE)
    go_to(base, BASE_RATIO, ready[0], BASE_MAX_ANGLE)

    # 2. Shoulder nod
    show("Demo: shoulder")
    go_to(shoulder, SHOULDER_RATIO, 60, SHOULDER_MAX_ANGLE)
    go_to(shoulder, SHOULDER_RATIO, 120, SHOULDER_MAX_ANGLE)
    go_to(shoulder, SHOULDER_RATIO, ready[1], SHOULDER_MAX_ANGLE)

    # 3. Elbow flex
    show("Demo: elbow")
    go_to(elbow, ELBOW_RATIO, 60, speed=ELBOW_MOVE_SPEED)
    go_to(elbow, ELBOW_RATIO, ready[2], speed=ELBOW_MOVE_SPEED)

    # 4. Wave: shoulder and elbow alternate, twice
    show("Demo: wave")
    for _ in range(2):
        go_pose(ready[0], 70, 90)
        go_pose(ready[0], 110, 135)

    # 5. Back to the ready pose
    go_pose(*ready)


def full_sequence():
    # 1. Home whichever joints are selected above
    home_selected()

    # 2. If the elbow wasn't homed above, whatever pose it is in right now becomes 0
    if not HOME_ELBOW:
        elbow.reset_angle(0)
    elbow.hold()  # stops it sagging under load

    show("Homed. Moving...")
    wait(1000)

    # 3. Small test moves, one joint at a time, returning to 0 after each
    show("Base")
    go_to(base, BASE_RATIO, TEST_BASE, BASE_MAX_ANGLE)
    go_to(base, BASE_RATIO, 0, BASE_MAX_ANGLE)

    show("Shoulder")
    go_to(shoulder, SHOULDER_RATIO, TEST_SHOULDER, SHOULDER_MAX_ANGLE)
    go_to(shoulder, SHOULDER_RATIO, 0, SHOULDER_MAX_ANGLE)

    show("Elbow")
    go_to(elbow, ELBOW_RATIO, TEST_ELBOW, speed=ELBOW_MOVE_SPEED)
    go_to(elbow, ELBOW_RATIO, 0, speed=ELBOW_MOVE_SPEED)

    # 4. Report where each joint ended up in arm degrees (should all be close to 0)
    show("A:{} B:{} C:{}".format(
        arm_angle(base, BASE_RATIO),
        arm_angle(shoulder, SHOULDER_RATIO),
        arm_angle(elbow, ELBOW_RATIO)))
    ev3.speaker.beep()
    wait(3000)


# ---------------------------------------------------------------
# REMOTE JOGGING
# ---------------------------------------------------------------
# One entry per joint: index 0 = base, 1 = shoulder, 2 = elbow.
# (motor, gear ratio, max arm angle or None, jog speed in joint deg/s)
JOINTS = [
    (base, BASE_RATIO, BASE_MAX_ANGLE, BASE_JOG_SPEED),
    (shoulder, SHOULDER_RATIO, SHOULDER_MAX_ANGLE, SHOULDER_JOG_SPEED),
    (elbow, ELBOW_RATIO, ELBOW_JOG_MAX, ELBOW_JOG_SPEED),
]

# What each joint is currently doing under remote control: +1, -1 or 0 (holding).
# Remembering this means hold() is only called once when a button is released,
# not every loop (calling hold() repeatedly keeps re-aiming at the sagging position).
jog_state = [0, 0, 0]


def jog_joint(i, direction):
    # direction: +1 = away from the stop, -1 = towards the stop, 0 = hold.
    # The usable range is REMOTE_LIMIT_MARGIN to (max angle - REMOTE_LIMIT_MARGIN).
    # A jog is a run_target() to the end of that range, so the motor's own
    # controller slows down and stops there. Checking the angle in this loop
    # alone is not enough: the loop is slow, so the joint would overshoot.
    motor, ratio, max_angle, speed = JOINTS[i]
    low = REMOTE_LIMIT_MARGIN
    high = None
    if max_angle is not None:
        high = max_angle - REMOTE_LIMIT_MARGIN

    angle = motor.angle() / ratio
    if direction > 0 and high is not None and angle >= high:
        direction = 0   # already at (or past) the top end: do nothing
    elif direction < 0 and angle <= low:
        direction = 0   # already at (or past) the bottom end: do nothing

    if direction != jog_state[i]:
        motor_speed = round(speed * ratio)
        if direction == 0:
            motor.hold()
        elif direction < 0:
            motor.run_target(motor_speed, round(low * ratio), then=Stop.HOLD, wait=False)
        elif high is not None:
            motor.run_target(motor_speed, round(high * ratio), then=Stop.HOLD, wait=False)
        else:
            motor.run(motor_speed)   # no upper limit set (only if ELBOW_JOG_MAX is None)
        jog_state[i] = direction


def stop_jogging():
    for i in range(3):
        jog_joint(i, 0)


def reset_jog_state():
    # Call after homing, the demo or the test sequence: those leave every
    # motor holding already, so the jog state starts again from "holding".
    for i in range(3):
        jog_state[i] = 0


def read_remote():
    # Returns (left_dir, right_dir, right_joint):
    #   left_dir  = +1 for LEFT_UP, -1 for LEFT_DOWN, 0 for neither or both
    #   right_dir = +1 for RIGHT_UP, -1 for RIGHT_DOWN, 0 for neither or both
    #   right_joint = which joint the right pair moves (1 = shoulder, 2 = elbow),
    #                 set by the channel the remote is currently sending on
    # The IR sensor listens on all four channels, so the channels in
    # REMOTE_CHANNELS are checked in turn and the first one with a button
    # pressed is used. With nothing pressed, everything holds.
    for channel, right_joint in REMOTE_CHANNELS:
        pressed = ir.buttons(channel)
        if len(pressed) > 0:
            left_dir = int(Button.LEFT_UP in pressed) - int(Button.LEFT_DOWN in pressed)
            right_dir = int(Button.RIGHT_UP in pressed) - int(Button.RIGHT_DOWN in pressed)
            return left_dir, right_dir, right_joint
    return 0, 0, 1


def show_mode():
    show("Remote switch:\n1 = base+shoulder\n2/3 = base+elbow\nTouch = home")


# ---------------------------------------------------------------
# START: wait for the centre button so nothing moves on upload
# ---------------------------------------------------------------
show("Centre button\nto start")
while Button.CENTER not in ev3.buttons.pressed():
    wait(10)
wait(500)
ev3.speaker.beep()

# ---------------------------------------------------------------
# MAIN LOOP: runs until you press the Back button on the brick
#   touch sensor (rising edge)  -> home all selected joints
#   remote (after homing)       -> jog the joints
#   brick UP (after homing)     -> run demo()
#   brick DOWN                  -> run full_sequence()
# ---------------------------------------------------------------
homed = False
prev_touch = touch.pressed()
prev_up = False
prev_down = False

show("Touch sensor:\nhome the arm")

while True:
    # 1. Touch sensor: act only on the moment it is pressed, not while it stays pressed
    touch_now = touch.pressed()
    if touch_now and not prev_touch:
        stop_jogging()
        home_selected()
        reset_jog_state()
        homed = True
        ev3.speaker.beep()
        show_mode()
        touch_now = touch.pressed()
    prev_touch = touch_now

    # 2. Brick buttons: UP = demo, DOWN = full test sequence
    pressed = ev3.buttons.pressed()
    up_now = Button.UP in pressed
    down_now = Button.DOWN in pressed

    if up_now and not prev_up:
        if homed:
            stop_jogging()
            demo()
            reset_jog_state()
            ev3.speaker.beep()
            show_mode()
        else:
            show("Home first:\npress touch sensor")
        up_now = Button.UP in ev3.buttons.pressed()

    if down_now and not prev_down:
        stop_jogging()
        full_sequence()
        reset_jog_state()
        homed = True
        show_mode()
        down_now = Button.DOWN in ev3.buttons.pressed()

    prev_up = up_now
    prev_down = down_now

    # 3. Remote control, only once the zero positions mean something
    if homed:
        left_dir, right_dir, right_joint = read_remote()

        # The joint the right pair is NOT pointing at gets 0, so it holds
        jog_joint(0, left_dir)
        if right_joint == 1:
            jog_joint(1, right_dir)
            jog_joint(2, 0)
        else:
            jog_joint(1, 0)
            jog_joint(2, right_dir)

    wait(20)