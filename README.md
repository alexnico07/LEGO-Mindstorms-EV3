# LEGO Mindstorms EV3

Projects built with the LEGO MINDSTORMS EV3 (31313) set, programmed in
MicroPython (pybricks) from VS Code.

## Projects

| Folder | Description |
|--------|-------------|
| `robot-arm/` | 3-joint robot arm (base, shoulder, elbow) with limit-stop homing |

## robot-arm

- **Motors:** A = base, B = shoulder, C = elbow
- **Gear ratios (motor : joint):** 3:1, 3:1, 1:1
- **Homing:** base and shoulder are driven slowly into their limit stops, and
  that position becomes 0. The elbow has no stop, so its start pose is 0.
- `arm_test.py`: homes the arm, then moves each joint out and back to 0.
- `simulate.py`: runs the script on a PC with a fake pybricks library and
  replays the motion in a window, so the logic can be checked without the brick.

Run the simulator:

    cd robot-arm
    py simulate.py arm_test.py

**Status:** simulated only, not yet tested on the real arm.

## Setup

1. Flash the EV3 MicroPython SD card image to a microSD card.
2. Install VS Code and the LEGO MINDSTORMS EV3 MicroPython extension.
3. Send the script to the brick from the extension's EV3 panel.
