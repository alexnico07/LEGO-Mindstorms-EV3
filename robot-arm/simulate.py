"""
Simulator for EV3 MicroPython arm scripts (no brick needed).

Usage:   py simulate.py [your_script.py] [--scenario | --no-visual]
         (default script: arm_test.py)

It replaces the pybricks library with a fake one that models each motor as a
shaft with hard stops. Your script runs unchanged. Needs tkinter for the
windows (it comes with Python on Windows).

Two ways to run it:

  Live (default)     Opens a window with a DIGITAL REMOTE: the IR remote with
                     its red channel switch and beacon button, the touch
                     sensor, and the brick's buttons and screen. You drive the
                     simulated arm yourself, in real time, and nothing runs
                     unless you press it (the demo only runs if you press the
                     brick's UP button). Close the window or click Back to stop
                     the program, like on the brick.

  --scenario         The SCENARIO list below plays the part of "you": it
                     presses the touch sensor and the remote's buttons on a
                     timeline, ending with the demo. When it is used up, a
                     window replays the motion. Good for quickly re-checking
                     the logic after an edit. --no-visual does the same without
                     the replay window.

What it models: the motors (moving over time, so run(), run_target(wait=False)
and run_until_stalled() all work), hard stops, the touch sensor, the IR sensor
with the remote's four channels, and the brick buttons. Your script's
"while True" loop is fine: in scenario mode the simulation ends once the
SCENARIO below is used up.

This checks the logic, direction handling, gear-ratio maths, limits and the
order of moves. It does NOT model real torque, real stall detection, gear
slip, motor acceleration, real timing, or collisions.
"""
import sys
import types
import runpy
import random
import bisect
import time
import threading
import traceback

# ---------------------------------------------------------------
# WORLD SETTINGS: edit these to describe your real arm
# ---------------------------------------------------------------
# "stop"  where the physical limit stop is, in the motor's own clockwise sense:
#         "low" = at the counter-clockwise end, "high" = at the clockwise end,
#         None = no stop.
#         To test a wrong Direction in your script, swap "low" and "high".
#         The elbow is "high" because ELBOW_DIR is COUNTERCLOCKWISE: homing
#         drives the motor clockwise into its stop.
# "ratio" motor turns per 1 turn of the joint. Used for the drawing and the
#         replay, so it should match the ratios in your script.
WORLD = {
    "A": {"name": "base",     "stop": "low",  "ratio": 3},
    "B": {"name": "shoulder", "stop": "low",  "ratio": 3},
    "C": {"name": "elbow",    "stop": "high", "ratio": 20 / 12},
}
NO_STALL_LIMIT = 720   # motor degrees of travel with no stall before the sim gives up
SEED = None            # set a number for repeatable random starting positions

# Time model
STEP = 0.01            # seconds per simulation step
SENSOR_READ_MS = 10    # simulated time one sensor read takes (a guess: measure your own)
TAIL_SECONDS = 1.0     # keep simulating this long after the scenario has finished
MAX_SIM_SECONDS = 900  # safety stop so a stuck script cannot run forever (scenario mode only)

# Live mode (the default)
MIN_PRESS_S = 0.3      # a click counts as a press at least this long (simulated seconds),
                       # because your script only looks at the buttons every so often

# ---------------------------------------------------------------
# SCENARIO: what "you" do while the simulation runs
# ---------------------------------------------------------------
# Actions run in order. An action starts the first time your script reads a
# sensor after the previous action finished, so a long homing run does not make
# the simulation miss a button press. Times are in seconds of simulated time.
#
#   ("pause", seconds)
#   ("touch", seconds)                       hold the touch sensor
#   ("remote", channel, buttons, seconds)    hold remote button(s) on a channel
#                                            (the red switch position, 1 to 4).
#                                            buttons is a name or a list of names:
#                                            LEFT_UP LEFT_DOWN RIGHT_UP RIGHT_DOWN BEACON
#   ("brick", button, seconds)               hold a brick button: UP DOWN LEFT RIGHT
#
# Only used with --scenario or --no-visual. The simulation ends TAIL_SECONDS
# after the last action. The default scenario homes the arm, then drives every
# joint to both ends of its range from the remote, then runs the demo.
SCENARIO = [
    ("pause", 0.5),
    ("touch", 0.3),                        # touch sensor: home the arm
    ("pause", 1.0),
    ("remote", 1, "LEFT_UP", 4.0),         # base up: should stop short of its max
    ("remote", 1, "LEFT_DOWN", 6.0),       # base down: should stop short of 0
    ("remote", 1, "RIGHT_UP", 4.0),        # switch 1: shoulder up
    ("remote", 1, "RIGHT_DOWN", 4.0),      # shoulder down
    ("remote", 2, "RIGHT_UP", 2.0),        # switch 2: elbow up
    ("remote", 3, "RIGHT_DOWN", 5.0),      # switch 3: elbow down: should stop short of 0
    ("pause", 1.0),
    ("brick", "UP", 0.3),                  # brick UP: run the demo
]

# Drawing settings (the drawing is a guess at your arm's shape, not the real thing)
UPPER_ARM_LEN = 120
FOREARM_LEN = 100


class SimulationEnd(Exception):
    pass


class _Clock:
    t = 0.0


EVENTS = []   # (time, text) for the replay captions
TRACK = {}    # joint name -> list of (time, arm angle from its stop in degrees)
MOTORS = []   # every motor the script has created


class _Live:
    # State shared between the simulation thread and the digital remote window.
    # Button names are "touch", "ir:LEFT_UP" (remote) and "brick:UP" (brick).
    on = False
    speed = 1.0       # simulated seconds per real second
    wall0 = 0.0       # real time at simulated time 0, used to pace the simulation
    stop = False      # set by the Back button or by closing the window
    status = ""       # message shown under the arm drawing
    channel = 1       # the red switch on the remote (1 to 4)
    beacon = False    # the remote's beacon button (latched on or off)
    down = set()      # buttons the mouse or keyboard is holding right now
    until = {}        # name -> simulated time until which it still counts as pressed
    screen = []       # lines on the brick's screen

    @classmethod
    def press(cls, name):
        cls.down.add(name)
        cls.until[name] = _Clock.t + MIN_PRESS_S

    @classmethod
    def release(cls, name):
        cls.down.discard(name)

    @classmethod
    def held(cls, name):
        return name in cls.down or _Clock.t < cls.until.get(name, -1.0)


def log(msg):
    print("   [{:6.2f}s] {}".format(_Clock.t, msg))
    EVENTS.append((_Clock.t, msg))


class _Port:
    def __init__(self, name):
        self.name = name


class Port:
    A = _Port("A")
    B = _Port("B")
    C = _Port("C")
    D = _Port("D")
    S1 = _Port("S1")
    S2 = _Port("S2")
    S3 = _Port("S3")
    S4 = _Port("S4")


class Direction:
    CLOCKWISE = "CLOCKWISE"
    COUNTERCLOCKWISE = "COUNTERCLOCKWISE"


class Stop:
    COAST = "COAST"
    BRAKE = "BRAKE"
    HOLD = "HOLD"


class Button:
    # brick buttons
    CENTER = "CENTER"
    UP = "UP"
    DOWN = "DOWN"
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    # infrared remote buttons
    LEFT_UP = "LEFT_UP"
    LEFT_DOWN = "LEFT_DOWN"
    RIGHT_UP = "RIGHT_UP"
    RIGHT_DOWN = "RIGHT_DOWN"
    BEACON = "BEACON"


# ---------------------------------------------------------------
# TIME
# ---------------------------------------------------------------
def _pace():
    # Live mode: sleep so that simulated time runs at _Live.speed times real time.
    target = _Live.wall0 + _Clock.t / _Live.speed
    delay = target - time.time()
    if delay > 0:
        time.sleep(delay)
    elif delay < -1.0:
        # we have fallen behind (slow PC or high speed): don't try to catch up
        _Live.wall0 = time.time() - _Clock.t / _Live.speed


def _set_live_speed(v):
    _Live.speed = v
    _Live.wall0 = time.time() - _Clock.t / v


def _advance(seconds):
    # Move time forward in small steps, moving every motor as it goes.
    remaining = seconds
    while remaining > 1e-9:
        if _Live.on and _Live.stop:
            raise SimulationEnd()
        dt = min(STEP, remaining)
        for m in MOTORS:
            m._step(dt)
        _Clock.t += dt
        remaining -= dt
        if _Live.on:
            _pace()
        elif _Clock.t > MAX_SIM_SECONDS:
            print("\nSimulation stopped: reached MAX_SIM_SECONDS ({}).".format(MAX_SIM_SECONDS))
            raise SimulationEnd()


def wait(ms):
    _advance(ms / 1000.0)


class StopWatch:
    def __init__(self):
        self.t0 = _Clock.t

    def time(self):
        return int((_Clock.t - self.t0) * 1000)

    def reset(self):
        self.t0 = _Clock.t


# ---------------------------------------------------------------
# MOTOR
# ---------------------------------------------------------------
class Motor:
    def __init__(self, port, positive_direction=Direction.CLOCKWISE, gears=None):
        cfg = WORLD[port.name]
        self.name = cfg["name"]
        self.ratio = cfg["ratio"]
        self.sign = 1 if positive_direction == Direction.CLOCKWISE else -1
        self.lo = None
        self.hi = None
        if cfg["stop"] == "low":
            self.lo = 0.0
            self.phys = random.uniform(100, 600)   # starts somewhere away from the stop
        elif cfg["stop"] == "high":
            self.hi = 0.0
            self.phys = -random.uniform(100, 600)
        else:
            self.phys = 0.0
        self.offset = 0.0
        # what the motor is doing: "hold" (not moving), "run" (at cmd_speed
        # until told otherwise) or "target" (moving to self.target, then hold)
        self.mode = "hold"
        self.cmd_speed = 0.0
        self.target = 0.0
        self._speed = 0.0
        TRACK[self.name] = [(0.0, self._vis())]
        MOTORS.append(self)
        log("{}: motor on port {} created, starts {} deg from its stop (physical)".format(
            self.name, port.name, "?" if cfg["stop"] is None else round(abs(self.phys))))

    # arm angle for the drawing: degrees away from the stop (true physical position)
    def _vis(self):
        if self.lo is not None:
            return (self.phys - self.lo) / self.ratio
        if self.hi is not None:
            return (self.hi - self.phys) / self.ratio
        return self.sign * self.phys / self.ratio

    # the motor's angle as a decimal, in the script's own sense
    def _raw(self):
        return self.sign * self.phys - self.offset

    # reported angle, as the real motor would report it
    def angle(self):
        return round(self._raw())

    def speed(self):
        return round(self._speed)

    def reset_angle(self, angle=None):
        a = 0 if angle is None else angle
        self.offset = self.sign * self.phys - a
        log("{}: reset_angle({})".format(self.name, a))

    def _step(self, dt):
        raw0 = self._raw()
        move = 0.0          # movement this step, in the script's own sense
        arrived = False
        if self.mode == "run":
            move = self.cmd_speed * dt
        elif self.mode == "target":
            delta = self.target - raw0
            step = abs(self.cmd_speed) * dt
            if abs(delta) <= step:
                move = delta
                arrived = True
            else:
                move = step if delta > 0 else -step

        hit = False
        if move != 0.0:
            new = self.phys + self.sign * move
            if self.lo is not None and new < self.lo:
                hit = (self.lo - new) > 1e-6
                new = self.lo
            if self.hi is not None and new > self.hi:
                hit = (new - self.hi) > 1e-6
                new = self.hi
            if new != self.phys:
                self.phys = new
                TRACK[self.name].append((_Clock.t + dt, self._vis()))

        if self.mode == "target":
            if hit:
                self.mode = "hold"
                log("{}: run_target({}) BLOCKED by a stop, ended at {} (wanted {})".format(
                    self.name, self.target, self.angle(), self.target))
            elif arrived:
                self.mode = "hold"
                log("{}: run_target({}) reached, motor angle {}".format(
                    self.name, self.target, self.angle()))
        self._speed = (self._raw() - raw0) / dt

    def run(self, speed):
        self.mode = "run"
        self.cmd_speed = speed
        log("{}: run({})".format(self.name, speed))

    def run_until_stalled(self, speed, then=Stop.COAST, duty_limit=None):
        d = self.sign * (1 if speed > 0 else -1)   # physical direction of travel
        limit = self.lo if d < 0 else self.hi
        if limit is None:
            self.mode = "run"
            self.cmd_speed = speed
            _advance(NO_STALL_LIMIT / abs(speed))
            log("{}: run_until_stalled({}) NEVER STALLED after {} motor deg".format(
                self.name, speed, NO_STALL_LIMIT))
            print("\nSIMULATION STOPPED: the {} joint drove away from its stop.".format(self.name))
            print("On the real arm it would keep going until something broke.")
            print("Fix: flip that joint's Direction in your script (CLOCKWISE <-> COUNTERCLOCKWISE).")
            sys.exit(1)
        self.mode = "run"
        self.cmd_speed = speed
        while abs(self.phys - limit) > 1e-9:
            _advance(STEP)
        self.mode = "hold"   # stalled against the stop; then=COAST means it just stays there
        log("{}: run_until_stalled({}) hit the stop, motor angle now {}".format(
            self.name, speed, self.angle()))
        return self.angle()

    def run_target(self, speed, target_angle, then=Stop.HOLD, wait=True):
        self.mode = "target"
        self.cmd_speed = abs(speed)
        self.target = target_angle
        if wait:
            while self.mode == "target":
                _advance(STEP)
        else:
            log("{}: run_target({}) started (not waiting)".format(self.name, target_angle))

    def hold(self):
        self.mode = "hold"

    def stop(self):
        self.mode = "hold"

    def brake(self):
        self.mode = "hold"


# ---------------------------------------------------------------
# SCENARIO PLAYER
# ---------------------------------------------------------------
class _Scenario:
    actions = []
    index = 0
    started = None    # clock time the current action began
    armed = False     # becomes True when the script first reads a sensor
    done_at = None


def _load_scenario():
    _Scenario.actions = []
    for item in SCENARIO:
        kind = item[0]
        act = {"kind": kind, "channel": None, "buttons": []}
        if kind in ("pause", "touch"):
            act["dur"] = item[1]
        elif kind == "remote":
            act["channel"] = item[1]
            names = item[2] if isinstance(item[2], (list, tuple)) else [item[2]]
            act["buttons"] = [getattr(Button, n) for n in names]
            act["dur"] = item[3]
        elif kind == "brick":
            act["buttons"] = [getattr(Button, item[1])]
            act["dur"] = item[2]
        else:
            raise ValueError("Unknown SCENARIO action: {}".format(item))
        _Scenario.actions.append(act)


def _describe(act):
    if act["kind"] == "touch":
        return "you press the touch sensor for {} s".format(act["dur"])
    if act["kind"] == "remote":
        return "you hold remote {} on channel {} for {} s".format(
            "+".join(act["buttons"]), act["channel"], act["dur"])
    return "you hold brick button {} for {} s".format(act["buttons"][0], act["dur"])


def _current_action():
    # The action that is active right now, or None once the scenario is used up.
    S = _Scenario
    now = _Clock.t
    while S.index < len(S.actions):
        act = S.actions[S.index]
        if S.started is None:
            S.started = now
            if act["kind"] != "pause":
                log(_describe(act))
        if now < S.started + act["dur"]:
            return act
        S.index += 1
        S.started = None
    return None


def _poll():
    # Every sensor read costs a little time, then looks at what "you" are doing.
    _advance(SENSOR_READ_MS / 1000.0)
    if _Live.on or not _Scenario.armed:
        return None
    act = _current_action()
    if act is None:
        if _Scenario.done_at is None:
            _Scenario.done_at = _Clock.t
        elif _Clock.t - _Scenario.done_at >= TAIL_SECONDS:
            raise SimulationEnd()
    return act


# ---------------------------------------------------------------
# SENSORS AND BRICK
# ---------------------------------------------------------------
class TouchSensor:
    def __init__(self, port):
        pass

    def pressed(self):
        _Scenario.armed = True   # the script is past its start-up wait: the scenario can begin
        act = _poll()
        if _Live.on:
            return _Live.held("touch")
        return act is not None and act["kind"] == "touch"


IR_BUTTON_NAMES = ("LEFT_UP", "LEFT_DOWN", "RIGHT_UP", "RIGHT_DOWN")


class InfraredSensor:
    def __init__(self, port):
        pass

    def buttons(self, channel):
        act = _poll()
        if _Live.on:
            if channel != _Live.channel:
                return []
            if _Live.beacon:
                # like the real remote: with the beacon on, no other buttons are reported
                return [Button.BEACON]
            return [getattr(Button, n) for n in IR_BUTTON_NAMES if _Live.held("ir:" + n)]
        if act is not None and act["kind"] == "remote" and act["channel"] == channel:
            return list(act["buttons"])
        return []


class _Screen:
    def clear(self):
        _Live.screen = []

    def print(self, *args, **kwargs):
        # the script's own print() already shows this in the terminal,
        # so just record it for the replay captions and the live brick screen
        text = " ".join(str(a) for a in args)
        EVENTS.append((_Clock.t, "Screen: " + text.replace("\n", " | ")))
        _Live.screen = _Live.screen + text.split("\n")


class _Speaker:
    def beep(self, *args, **kwargs):
        log("beep")
        _advance(0.1)


class _Buttons:
    def pressed(self):
        if _Live.on:
            return [getattr(Button, n) for n in ("CENTER", "UP", "DOWN", "LEFT", "RIGHT")
                    if _Live.held("brick:" + n)]
        # scenario mode: the centre button counts as pressed straight away;
        # other brick buttons come from the scenario
        result = [Button.CENTER]
        if _Scenario.armed:
            act = _current_action()
            if act is not None and act["kind"] == "brick":
                result += act["buttons"]
        return result


class EV3Brick:
    def __init__(self):
        self.screen = _Screen()
        self.speaker = _Speaker()
        self.buttons = _Buttons()


def _install_fake_pybricks():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m
    root = mod("pybricks")
    root.hubs = mod("pybricks.hubs", EV3Brick=EV3Brick)
    root.ev3devices = mod("pybricks.ev3devices", Motor=Motor, TouchSensor=TouchSensor,
                          InfraredSensor=InfraredSensor)
    root.parameters = mod("pybricks.parameters", Port=Port, Direction=Direction,
                          Stop=Stop, Button=Button)
    root.tools = mod("pybricks.tools", wait=wait, StopWatch=StopWatch)


# ---------------------------------------------------------------
# ARM DRAWING (shared by the replay and the live window)
# ---------------------------------------------------------------
def draw_arm(canvas, W, b, s, e, caption_text, t):
    # Draws the arm: top view of the base, side view of shoulder and elbow.
    # b, s and e are arm degrees away from each joint's stop.
    import math
    canvas.delete("all")

    # top view: the base turning
    cx, cy, R = 210, 205, 130
    canvas.create_text(cx, 25, text="TOP VIEW (base)", font=("Arial", 12, "bold"))
    canvas.create_oval(cx - R, cy - R, cx + R, cy + R, outline="#bbbbbb")
    canvas.create_line(cx + R - 12, cy, cx + R + 18, cy, fill="red", width=5)
    canvas.create_text(cx + R + 4, cy - 14, text="stop = 0", fill="red", font=("Arial", 9))
    a = math.radians(b)
    canvas.create_line(cx, cy, cx + (R - 10) * math.cos(a), cy - (R - 10) * math.sin(a),
                       fill="#1f6feb", width=7, capstyle="round")
    canvas.create_oval(cx - 8, cy - 8, cx + 8, cy + 8, fill="#444444")

    # side view: shoulder and elbow
    px, py = 640, 330
    canvas.create_text(640, 25, text="SIDE VIEW (shoulder + elbow)", font=("Arial", 12, "bold"))
    canvas.create_rectangle(px - 45, py + 8, px + 45, py + 50, fill="#dddddd", outline="#999999")
    canvas.create_line(px, py, px + UPPER_ARM_LEN + 30, py, fill="red", dash=(4, 3))
    canvas.create_text(px + UPPER_ARM_LEN + 55, py - 10, text="shoulder 0 (stop)",
                       fill="red", font=("Arial", 9))
    al = math.radians(s)
    x1 = px + UPPER_ARM_LEN * math.cos(al)
    y1 = py - UPPER_ARM_LEN * math.sin(al)
    be = math.radians(s + e)
    x2 = x1 + FOREARM_LEN * math.cos(be)
    y2 = y1 - FOREARM_LEN * math.sin(be)
    canvas.create_line(px, py, x1, y1, fill="#1f6feb", width=9, capstyle="round")
    canvas.create_line(x1, y1, x2, y2, fill="#e8590c", width=7, capstyle="round")
    for (x, y) in ((px, py), (x1, y1)):
        canvas.create_oval(x - 7, y - 7, x + 7, y + 7, fill="#444444")
    canvas.create_oval(x2 - 5, y2 - 5, x2 + 5, y2 + 5, fill="#2b8a3e")

    canvas.create_text(W / 2, 372,
                       text="base {:.0f}\u00b0     shoulder {:.0f}\u00b0     elbow {:.0f}\u00b0    (arm degrees)".format(b, s, e),
                       font=("Arial", 12))
    canvas.create_text(W / 2, 398, text=caption_text, font=("Arial", 10), fill="#333333")
    canvas.create_text(50, 25, text="t = {:.1f} s".format(t), font=("Arial", 11))


# ---------------------------------------------------------------
# REPLAY WINDOW
# ---------------------------------------------------------------
def show_visual(end_time):
    import math
    import tkinter as tk

    end_time = max(end_time, 1.0)
    root = tk.Tk()
    root.title("Arm simulation replay")
    W, H = 900, 430
    canvas = tk.Canvas(root, width=W, height=H, bg="white")
    canvas.pack()
    state = {"t": 0.0, "playing": True, "speed": 1.0}

    # the tracks can have thousands of points, so look values up by bisection
    times = {name: [p[0] for p in pts] for name, pts in TRACK.items()}

    def value(name, t):
        pts = TRACK.get(name)
        if not pts:
            return 0.0
        if t <= pts[0][0]:
            return pts[0][1]
        k = bisect.bisect_right(times[name], t)
        if k >= len(pts):
            return pts[-1][1]
        t0, a0 = pts[k - 1]
        t1, a1 = pts[k]
        if t1 == t0:
            return a1
        return a0 + (a1 - a0) * (t - t0) / (t1 - t0)

    def caption(t):
        text = ""
        for te, msg in EVENTS:
            if te <= t + 1e-9:
                text = msg
            else:
                break
        return text

    def draw():
        t = state["t"]
        draw_arm(canvas, W, value("base", t), value("shoulder", t), value("elbow", t),
                 caption(t), t)

    controls = tk.Frame(root)
    controls.pack(fill="x", padx=8, pady=4)

    def toggle():
        if state["t"] >= end_time:
            state["t"] = 0.0
            state["playing"] = True
        else:
            state["playing"] = not state["playing"]
        play_btn.config(text="Pause" if state["playing"] else "Play")

    def restart():
        state["t"] = 0.0
        state["playing"] = True
        play_btn.config(text="Pause")

    def set_speed(v):
        state["speed"] = v

    def on_scale(v):
        if abs(float(v) - state["t"]) < 0.06:
            return   # the slider is just following playback, not being dragged
        state["t"] = float(v)
        state["playing"] = False
        play_btn.config(text="Play")
        draw()

    play_btn = tk.Button(controls, text="Pause", width=7, command=toggle)
    play_btn.pack(side="left")
    tk.Button(controls, text="Restart", width=7, command=restart).pack(side="left", padx=4)
    for label, v in (("0.5x", 0.5), ("1x", 1.0), ("2x", 2.0), ("4x", 4.0), ("8x", 8.0)):
        tk.Button(controls, text=label, width=4, command=lambda v=v: set_speed(v)).pack(side="left")
    scale = tk.Scale(controls, from_=0, to=end_time, resolution=0.05, orient="horizontal",
                     showvalue=False, length=520, command=on_scale)
    scale.pack(side="left", padx=10)

    def tick():
        if state["playing"]:
            state["t"] = min(end_time, state["t"] + 0.03 * state["speed"])
            if state["t"] >= end_time:
                state["playing"] = False
                play_btn.config(text="Play")
            scale.set(state["t"])
        draw()
        root.after(30, tick)

    tick()
    root.mainloop()


# ---------------------------------------------------------------
# LIVE WINDOW WITH THE DIGITAL REMOTE (the default)
# ---------------------------------------------------------------
def run_live(script):
    import tkinter as tk

    _Live.on = True
    _Live.wall0 = time.time()

    root = tk.Tk()
    root.title("Arm simulator - digital remote")
    W, H = 900, 430
    canvas = tk.Canvas(root, width=W, height=H, bg="white")
    canvas.pack()
    panel = tk.Frame(root)
    panel.pack(fill="x", padx=8, pady=4)

    def hold_button(parent, text, name, **kw):
        # a button that counts as pressed for as long as the mouse is held on it
        b = tk.Button(parent, text=text, **kw)
        b.bind("<ButtonPress-1>", lambda ev: _Live.press(name))
        b.bind("<ButtonRelease-1>", lambda ev: _Live.release(name))
        return b

    # --- the IR remote ---
    remote = tk.LabelFrame(panel, text="IR remote")
    remote.pack(side="left", padx=6)
    chan = tk.IntVar(value=_Live.channel)

    def set_channel():
        _Live.channel = chan.get()
        log("remote: red switch moved to {}".format(_Live.channel))

    switch = tk.Frame(remote)
    switch.grid(row=0, column=0, columnspan=2, pady=2)
    tk.Label(switch, text="Red switch:").pack(side="left")
    for n in (1, 2, 3, 4):
        tk.Radiobutton(switch, text=str(n), variable=chan, value=n, indicatoron=0, width=3,
                       selectcolor="#e03131", command=set_channel).pack(side="left")
    hold_button(remote, "\u25b2 Left", "ir:LEFT_UP", width=9).grid(row=1, column=0, padx=2, pady=1)
    hold_button(remote, "\u25b2 Right", "ir:RIGHT_UP", width=9).grid(row=1, column=1, padx=2, pady=1)
    hold_button(remote, "\u25bc Left", "ir:LEFT_DOWN", width=9).grid(row=2, column=0, padx=2, pady=1)
    hold_button(remote, "\u25bc Right", "ir:RIGHT_DOWN", width=9).grid(row=2, column=1, padx=2, pady=1)
    beacon_var = tk.IntVar(value=0)

    def set_beacon():
        _Live.beacon = bool(beacon_var.get())
        log("remote: beacon {}".format("ON" if _Live.beacon else "off"))

    tk.Checkbutton(remote, text="Beacon (latches on)", variable=beacon_var,
                   command=set_beacon).grid(row=3, column=0, columnspan=2)

    # --- the touch sensor ---
    touch_frame = tk.LabelFrame(panel, text="Touch sensor (port 1)")
    touch_frame.pack(side="left", padx=6, fill="y")
    hold_button(touch_frame, "Press", "touch", width=12, height=4).pack(padx=6, pady=8)

    # --- the brick's buttons and screen ---
    brick = tk.LabelFrame(panel, text="EV3 brick")
    brick.pack(side="left", padx=6)
    hold_button(brick, "Up", "brick:UP", width=8).grid(row=0, column=1)
    hold_button(brick, "Centre", "brick:CENTER", width=8).grid(row=1, column=1)
    hold_button(brick, "Down", "brick:DOWN", width=8).grid(row=2, column=1)

    def back():
        _Live.stop = True
        _Live.status = "Back pressed: the program was stopped."

    tk.Button(brick, text="Back (stop)", width=10, command=back).grid(row=1, column=0, padx=4)
    screen = tk.Label(brick, text="", font=("Courier", 10), width=22, height=6, bg="#c9d6b0",
                      justify="left", anchor="nw", relief="sunken")
    screen.grid(row=0, column=2, rowspan=3, padx=6, pady=2)

    # --- simulation speed ---
    speed_frame = tk.LabelFrame(panel, text="Speed")
    speed_frame.pack(side="left", padx=6, fill="y")
    for label, v in (("0.5x", 0.5), ("1x", 1.0), ("2x", 2.0), ("4x", 4.0)):
        tk.Button(speed_frame, text=label, width=5, command=lambda v=v: _set_live_speed(v)).pack(pady=1)

    tk.Label(root, anchor="w", justify="left", fg="#555555", font=("Arial", 9),
             text="Keys: W / S = remote left up / down,  Up / Down arrows = remote right up / down "
                  "(hold two to press both),  T = touch sensor,  1-4 = red switch,  Enter = brick Centre.  "
                  "Close the window to stop.").pack(fill="x", padx=8)

    # keyboard: lets you hold two remote buttons at once, which the mouse cannot do
    KEYS = {"w": "ir:LEFT_UP", "s": "ir:LEFT_DOWN", "Up": "ir:RIGHT_UP", "Down": "ir:RIGHT_DOWN",
            "t": "touch", "Return": "brick:CENTER"}

    def key_name(ev):
        return ev.keysym.lower() if len(ev.keysym) == 1 else ev.keysym

    def key_down(ev):
        k = key_name(ev)
        if k in KEYS:
            _Live.press(KEYS[k])
        elif k in ("1", "2", "3", "4"):
            chan.set(int(k))
            set_channel()

    def key_up(ev):
        k = key_name(ev)
        if k in KEYS:
            _Live.release(KEYS[k])

    root.bind("<KeyPress>", key_down)
    root.bind("<KeyRelease>", key_up)

    def worker():
        try:
            runpy.run_path(script, run_name="__main__")
            _Live.status = "The script finished by itself."
        except SimulationEnd:
            if not _Live.status:
                _Live.status = "Stopped."
        except SystemExit:
            _Live.status = "The script stopped (see the terminal for why)."
        except Exception:
            traceback.print_exc()
            _Live.status = "The script crashed (see the terminal)."

    threading.Thread(target=worker, daemon=True).start()

    def tick():
        vis = {m.name: m._vis() for m in MOTORS}
        caption = _Live.status or (EVENTS[-1][1] if EVENTS else "")
        draw_arm(canvas, W, vis.get("base", 0.0), vis.get("shoulder", 0.0), vis.get("elbow", 0.0),
                 caption, _Clock.t)
        screen.config(text="\n".join(_Live.screen[-6:]))
        root.after(30, tick)

    def on_close():
        _Live.stop = True
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    tick()
    root.mainloop()
    _Live.stop = True


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    visual = "--no-visual" not in sys.argv
    scripted = "--scenario" in sys.argv or "--no-visual" in sys.argv
    script = args[0] if args else "arm_test.py"
    if SEED is not None:
        random.seed(SEED)
    if not scripted:
        _install_fake_pybricks()
        print("Live simulation of {} (close the window to stop) ...\n".format(script))
        try:
            run_live(script)
        except ImportError:
            print("The live window needs tkinter, which isn't available here. Use --scenario or --no-visual.")
            sys.exit(1)
        sys.exit(0)
    _load_scenario()
    _install_fake_pybricks()
    print("Simulating {} ...\n".format(script))
    exit_code = 0
    try:
        runpy.run_path(script, run_name="__main__")
        print("\nSimulation finished. Simulated run time: {:.1f} s".format(_Clock.t))
    except SimulationEnd:
        print("\nScenario finished. Simulated run time: {:.1f} s".format(_Clock.t))
    except SystemExit as e:
        exit_code = e.code if isinstance(e.code, int) else 1
    if MOTORS:
        print("Final joint angles (arm degrees from each stop): " + ", ".join(
            "{} {:.1f}".format(m.name, m._vis()) for m in MOTORS))
    if visual:
        try:
            show_visual(_Clock.t)
        except ImportError:
            print("(tkinter isn't available, so no replay window. Use --no-visual to hide this.)")
    sys.exit(exit_code)