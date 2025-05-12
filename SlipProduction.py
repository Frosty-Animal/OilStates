import cv2                            # OpenCV for image processing
import numpy as np                    # numerical operations
from picamera2 import Picamera2      # Raspberry Pi camera interface
import time                           # timing functions
import pigpio                         # hardware-timed pulses
import RPi.GPIO as GPIO               # GPIO control
from collections import namedtuple    # lightweight data structures

# ——— Pin Definitions ———
LimitSwitch_M1 = 25   # endstop for X‑axis
LimitSwitch_M2 = 24   # endstop for Y‑axis
LimitSwitch_M3 = 23   # endstop for Z‑axis
Belt_Stopped    = 37  # input from belt encoder to signal “stopped”

StepPin_1 = 18  # step pulse for X
StepPin_2 = 12  # step pulse for Y
StepPin_3 = 20  # step pulse for Z
DirPin_1  = 19  # direction control for X
DirPin_2  = 13  # direction control for Y
DirPin_3  = 21  # direction control for Z

CW_Direction  = 0          # constant for clockwise (or positive) direction
CCW_Direction = 1          # constant for counterclockwise (or negative)
Pulse_width_us = 3         # high-pulse duration in microseconds
STEPS_PER_UNIT = 1         # units→ step counts

# define a Step record\ nStep = namedtuple('Step', ['dx', 'dy', 'delay'])

# ——— Setup ———
GPIO.setmode(GPIO.BCM)
for pin in (DirPin_1, DirPin_2, DirPin_3, StepPin_1, StepPin_2, StepPin_3):
    GPIO.setup(pin, GPIO.OUT)
GPIO.setup(Belt_Stopped, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
for sw in (LimitSwitch_M1, LimitSwitch_M2, LimitSwitch_M3):
    GPIO.setup(sw, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

class StepperSystem:
    def __init__(self):
        self.pi = pigpio.pi()
        self.axes = {
            'x': {'step': StepPin_1, 'dir': DirPin_1},
            'y': {'step': StepPin_2, 'dir': DirPin_2},
            'z': {'step': StepPin_3, 'dir': DirPin_3},
        }
        self.limits = {'x': LimitSwitch_M1, 'y': LimitSwitch_M2, 'z': LimitSwitch_M3}
        # camera
        self.picam = Picamera2()
        self.picam.start()

    # Bresenham line in Δ-steps
    def bresenham_steps(self, dx, dy):
        sx = 1 if dx >= 0 else -1
        sy = 1 if dy >= 0 else -1
        dx_a, dy_a = abs(dx), abs(dy)
        err = dx_a - dy_a
        x = y = 0
        path = []
        while (x, y) != (dx, dy):
            e2 = err * 2
            sx_step = sy_step = 0
            if e2 > -dy_a:
                err -= dy_a
                x += sx
                sx_step = sx
            if e2 < dx_a:
                err += dx_a
                y += sy
                sy_step = sy
            path.append((sx_step, sy_step))
        return path

    # trapezoidal timing
    def trapezoidal_durations(self, n, v_max, accel):
        if n == 0:
            return []
        n_a = int(v_max ** 2 / (2 * accel))
        if 2 * n_a > n:
            n_a = n // 2
            v_peak = (accel * n) ** 0.5
        else:
            v_peak = v_max
        n_c = n - 2 * n_a
        dts = []
        # acceleration phase
        for i in range(1, n_a + 1):
            dts.append(1 / ((2 * accel * i) ** 0.5))
        # constant speed phase
        dts += [1 / v_peak] * n_c
        # deceleration phase
        for i in range(n_a, 0, -1):
            dts.append(1 / ((2 * accel * i) ** 0.5))
        return dts

    # bundle into Step objects
    def make_steps(self, dx, dy, v_max, accel):
        deltas = self.bresenham_steps(dx, dy)
        dts = self.trapezoidal_durations(len(deltas), v_max, accel)
        return [Step(dx=sdx, dy=sdy, delay=dt) for (sdx, sdy), dt in zip(deltas, dts)]

    # low-level waveform step
    def step_parallel(self, steps, pulse_us=Pulse_width_us):
        for ax, cnt in steps.items():
            direction = CW_Direction if cnt >= 0 else CCW_Direction
            self.pi.write(self.axes[ax]['dir'], direction)
        total = max(abs(c) for c in steps.values())
        if total == 0:
            return
        counts = {ax: abs(c) for ax, c in steps.items()}
        err = {ax: 0 for ax in steps}
        pulses = []
        for _ in range(total):
            for ax in steps:
                err[ax] += counts[ax]
                if err[ax] >= total:
                    pin = self.axes[ax]['step']
                    pulses.append(pigpio.pulse(1 << pin, 0, pulse_us))
                    pulses.append(pigpio.pulse(0, 1 << pin, pulse_us))
                    err[ax] -= total
        self.pi.wave_add_generic(pulses)
        wid = self.pi.wave_create()
        self.pi.wave_send_once(wid)
        while self.pi.wave_tx_busy():
            pass
        self.pi.wave_delete(wid)

    # homing
    def home(self):
        for ax in self.axes:
            self.pi.write(self.axes[ax]['dir'], CCW_Direction)
        homed = {ax: False for ax in self.axes}
        while not all(homed.values()):
            for ax in self.axes:
                if not homed[ax]:
                    self.step_parallel({ax: -1})
                    if GPIO.input(self.limits[ax]):
                        homed[ax] = True
        return True

    # pickup
    def pickup(self, steps=100):
        self.step_parallel({'z': -steps})
        self.step_parallel({'z': steps})

    # place using plan
    def place(self, results, v_max=800, accel=2000):
        for _, dx, dy in results:
            sx = int(dx * STEPS_PER_UNIT)
            sy = int(dy * STEPS_PER_UNIT)
            plan = self.make_steps(sx, sy, v_max, accel)
            for step in plan:
                self.step_parallel({'x': step.dx, 'y': step.dy, 'z': 0})
                time.sleep(step.delay)
            self.pickup()

    # vision
    def capture_and_detect(self, retries=5, delay=0.5):
        for _ in range(retries):
            try:
                frame = self.picam.capture_array()
                break
            except RuntimeError:
                time.sleep(delay)
        else:
            return None
        img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        gray = cv2.medianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 11)
        blk = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, 51, -2
        )
        circles = cv2.HoughCircles(
            blk, cv2.HOUGH_GRADIENT, 1, 75,
            param1=110, param2=13,
            minRadius=25, maxRadius=37
        )
        if circles is None or len(circles[0]) < 4:
            return None
        pts = sorted(
            [(c[0], c[1]) for c in
             np.uint16(np.around(circles))[0, :4]],
            key=lambda p: (p[1], p[0])
        )
        def dist(a, b):
            return np.linalg.norm(np.array(a) - np.array(b))
        res = []
        for i in range(4):
            a = pts[i]
            b = pts[(i + 1) % 4]
            d = dist(a, b)
            res.append((d, b[0] - a[0], b[1] - a[1]))
        return tuple(res)

    # main
    def main(self):
        homed = False
        while True:
            if not homed:
                homed = self.home()
            elif GPIO.input(Belt_Stopped):
                results = self.capture_and_detect()
                if results:
                    self.place(results)
                    bx = -sum(r[1] for r in results)
                    by = -sum(r[2] for r in results)
                    plan = self.make_steps(bx, by, 800, 2000)
                    for step in plan:
                        self.step_parallel(
                            {'x': step.dx, 'y': step.dy, 'z': 0}
                        )
                        time.sleep(step.delay)
            time.sleep(0.1)

if __name__ == '__main__':
    system = StepperSystem()
    system.main()
