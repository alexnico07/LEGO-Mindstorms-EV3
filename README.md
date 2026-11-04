# LEGO Mindstorms EV3

Projects built with the LEGO MINDSTORMS EV3 (31313) set, programmed in
MicroPython (pybricks) from VS Code.

## Projects

| Folder | Description |
|--------|-------------|
| `robot-arm/` | 3-joint robot arm (base, shoulder, elbow) with limit-stop homing, touch-sensor homing and IR remote control |

## robot-arm

- **Motors:** A = base, B = shoulder, C = elbow
- **Sensors:** touch sensor on port 1, IR sensor on port 4 (EV3 infrared remote)
- **Gear ratios (motor : joint):** base 3:1, shoulder 3:1, elbow 20:12
- **Homing:** every joint is driven slowly into its limit stop, and that
  position becomes 0. Each joint then turns back to a ready pose. The base and
  shoulder stop at fixed maximum angles (base 200, shoulder 145). The elbow has
  no maximum angle yet.
- **Speeds** are in degrees per second at the joint. The code multiplies by each
  joint's gear ratio to get the motor speed.

### Files

- `arm_test.py`: the control script. It waits for the centre button, then runs
  until you press Back on the brick.
- `simulate.py`: runs the script on a PC with a fake pybricks library and
  replays the motion in a window, so the logic can be checked without the brick.

### Controls

| Input | What it does |
|-------|--------------|
| Centre button | Starts the program (nothing moves before this) |
| Touch sensor | Homes the arm. Press it first: the remote only works after homing |
| Remote, red switch on 1 | Left buttons move the base, right buttons move the shoulder |
| Remote, red switch on 2 or 3 | Left buttons move the base, right buttons move the elbow |
| Brick UP | Runs the demo routine (after homing) |
| Brick DOWN | Runs the full test sequence |
| Back | Stops the program |

While jogging with the remote, each joint stops 5 degrees short of both ends of
its range so it never drives into a stop.

### Simulator

    cd robot-arm
    py simulate.py arm_test.py

This opens a window with the arm, an on-screen IR remote (red channel switch,
four buttons and the beacon button), the touch sensor, the brick's buttons and
screen, and a speed control. You drive the simulated arm in real time, and
nothing runs unless you press it. Keyboard shortcuts let you hold two remote
buttons at once: W / S for the left pair, the Up / Down arrows for the right
pair, T for the touch sensor, 1 to 4 for the red switch and Enter for the
brick's Centre button. Close the window or click Back to stop.

To run a scripted sequence instead, use:

    py simulate.py arm_test.py --scenario

This follows the `SCENARIO` list at the top of `simulate.py` (touch press,
remote buttons, brick buttons, then the demo), then opens a window that replays
the motion. Edit the list to try other sequences. Add `--no-visual` to skip the
window.

The simulator checks logic, directions, gear-ratio maths and limits. It does not
model real torque, gear slip, motor acceleration or collisions.

### Project status

- **Gripper:** waiting on another motor for the gripper, so the arm has no
  gripper yet.
- **Building instructions:** designing the building instructions for the arm.
- **Testing:** work in progress. Homing and remote jogging have been tried on
  the real arm. The demo routine and the new 20:12 elbow gearing still need to
  be checked, and the base and shoulder gear ratios are assumed rather than
  measured.

**Known issue:** before the elbow was rebuilt, it stopped at about 90 degrees
when asked to turn back 135 degrees. The new gearing was added to try to fix
this, and the cause has not been confirmed.

## Setup

1. Flash the EV3 MicroPython SD card image to a microSD card and put it in the brick.
2. Install VS Code and the EV3DEV Device Browser extension, then connect to the
   brick over USB.
3. In `launch.json`, set `program` to
   `/home/robot/${workspaceRootFolderName}/robot-arm/arm_test.py`.
4. Press F5 ("Download and Run"). Keep a finger on the Back button on the first
   run of anything new.