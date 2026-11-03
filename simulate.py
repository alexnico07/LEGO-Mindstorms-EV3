"""
Simulator for EV3 MicroPython arm scripts (no brick needed).

Usage:   py simulate.py [your_script.py] [--no-visual]
         (default script: arm_test.py)

It replaces the pybricks library with a fake one that models each motor as a
shaft with hard stops. Your script runs unchanged. When it finishes, a window
opens that replays the motion (needs tkinter, which comes with Python on Windows).

This checks the logic, direction handling, gear-ratio maths and the order of
moves. It does NOT model real torque, real stall detection, gear slip, real
timing, or collisions.
"""
import sys
import types
import runpy
import random

# ---------------------------------------------------------------
# WORLD SETTINGS: edit these to describe your real arm
# ---------------------------------------------------------------
# "stop"  where the physical limit stop is, in the motor's own clockwise sense:
#         "low" = at the counter-clockwise end, "high" = at the clockwise end,
#         None = no stop (the elbow).
#         To test a wrong Direction in your script, swap "low" and "high".
# "ratio" motor turns per 1 turn of the joint. Only used for the drawing and
#         should match the ratios in your script.
WORLD = {
    "A": {"name": "base",     "stop": "low", "ratio": 3},
    "B": {"name": "shoulder", "stop": "low", "ratio": 3},
    "C": {"name": "elbow",    "stop": None,  "ratio": 1},
}
NO_STALL_LIMIT = 720   # motor degrees of travel with no stall before the sim gives up
SEED = None            # set a number for repeatable random starting positions

# Drawing settings (the drawing is a guess at your arm's shape, not the real thing)
UPPER_ARM_LEN = 120
FOREARM_LEN = 100


class _Clock:
    t = 0.0


EVENTS = []   # (time, text) for the replay captions
TRACK = {}    # joint name -> list of (time, arm angle from its stop in degrees)


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


class Direction:
    CLOCKWISE = "CLOCKWISE"
    COUNTERCLOCKWISE = "COUNTERCLOCKWISE"


class Stop:
    COAST = "COAST"
    BRAKE = "BRAKE"
    HOLD = "HOLD"


class Button:
    CENTER = "CENTER"
    UP = "UP"
    DOWN = "DOWN"
    LEFT = "LEFT"
    RIGHT = "RIGHT"


def wait(ms):
    _Clock.t += ms / 1000.0


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
        TRACK[self.name] = [(0.0, self._vis())]
        log("{}: motor on port {} created, starts {} deg from its stop (physical)".format(
            self.name, port.name, "?" if cfg["stop"] is None else round(abs(self.phys))))

    # arm angle for the drawing: degrees away from the stop (true physical position)
    def _vis(self):
        if self.lo is not None:
            return (self.phys - self.lo) / self.ratio
        if self.hi is not None:
            return (self.hi - self.phys) / self.ratio
        return self.sign * self.phys / self.ratio

    # reported angle, as the real motor would report it
    def angle(self):
        return round(self.sign * self.phys - self.offset)

    def reset_angle(self, angle=None):
        a = 0 if angle is None else angle
        self.offset = self.sign * self.phys - a
        log("{}: reset_angle({})".format(self.name, a))

    def _travel(self, dphys, speed):
        TRACK[self.name].append((_Clock.t, self._vis()))
        new = self.phys + dphys
        stalled = False
        if self.lo is not None and new < self.lo:
            new = self.lo
            stalled = True
        if self.hi is not None and new > self.hi:
            new = self.hi
            stalled = True
        actual = new - self.phys
        _Clock.t += abs(actual) / abs(speed)
        self.phys = new
        TRACK[self.name].append((_Clock.t, self._vis()))
        return stalled

    def run_until_stalled(self, speed, then=Stop.COAST, duty_limit=None):
        d = self.sign * (1 if speed > 0 else -1)   # physical direction of travel
        limit = self.lo if d < 0 else self.hi
        if limit is None:
            self._travel(d * NO_STALL_LIMIT, speed)
            log("{}: run_until_stalled({}) NEVER STALLED after {} motor deg".format(
                self.name, speed, NO_STALL_LIMIT))
            print("\nSIMULATION STOPPED: the {} joint drove away from its stop.".format(self.name))
            print("On the real arm it would keep going until something broke.")
            print("Fix: flip that joint's Direction in your script (CLOCKWISE <-> COUNTERCLOCKWISE).")
            sys.exit(1)
        self._travel(limit - self.phys, speed)
        log("{}: run_until_stalled({}) hit the stop, motor angle now {}".format(
            self.name, speed, self.angle()))
        return self.angle()

    def run_target(self, speed, target_angle, then=Stop.HOLD, wait=True):
        delta = target_angle - self.angle()
        stalled = self._travel(self.sign * delta, speed)
        if stalled:
            log("{}: run_target({}) BLOCKED by a stop, ended at {} (wanted {})".format(
                self.name, target_angle, self.angle(), target_angle))
        else:
            log("{}: run_target({}) reached, motor angle {}".format(
                self.name, target_angle, self.angle()))

    def hold(self):
        pass

    def stop(self):
        pass

    def brake(self):
        pass


class _Screen:
    def clear(self):
        pass

    def print(self, *args, **kwargs):
        # the script's own print() already shows this in the terminal,
        # so just record it for the replay captions
        EVENTS.append((_Clock.t, "Screen: " + " ".join(str(a) for a in args).replace("\n", " | ")))


class _Speaker:
    def beep(self, *args, **kwargs):
        log("beep")


class _Buttons:
    def pressed(self):
        return [Button.CENTER]   # pretend the centre button is pressed straight away


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
    root.ev3devices = mod("pybricks.ev3devices", Motor=Motor)
    root.parameters = mod("pybricks.parameters", Port=Port, Direction=Direction,
                          Stop=Stop, Button=Button)
    root.tools = mod("pybricks.tools", wait=wait)


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

    def value(name, t):
        pts = TRACK.get(name)
        if not pts:
            return 0.0
        if t <= pts[0][0]:
            return pts[0][1]
        for (t0, a0), (t1, a1) in zip(pts, pts[1:]):
            if t0 <= t <= t1:
                if t1 == t0:
                    return a1
                return a0 + (a1 - a0) * (t - t0) / (t1 - t0)
        return pts[-1][1]

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
        b = value("base", t)
        s = value("shoulder", t)
        e = value("elbow", t)
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
        canvas.create_text(W / 2, 398, text=caption(t), font=("Arial", 10), fill="#333333")
        canvas.create_text(50, 25, text="t = {:.1f} s".format(t), font=("Arial", 11))

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
    for label, v in (("0.5x", 0.5), ("1x", 1.0), ("2x", 2.0), ("4x", 4.0)):
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


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    visual = "--no-visual" not in sys.argv
    script = args[0] if args else "arm_test.py"
    if SEED is not None:
        random.seed(SEED)
    _install_fake_pybricks()
    print("Simulating {} ...\n".format(script))
    exit_code = 0
    try:
        runpy.run_path(script, run_name="__main__")
        print("\nSimulation finished. Simulated run time: {:.1f} s".format(_Clock.t))
    except SystemExit as e:
        exit_code = e.code if isinstance(e.code, int) else 1
    if visual:
        try:
            show_visual(_Clock.t)
        except ImportError:
            print("(tkinter isn't available, so no replay window. Use --no-visual to hide this.)")
    sys.exit(exit_code)