import cv2                            # OpenCV for image processing
import numpy as np                    # numerical operations
from picamera2 import Picamera2      # Raspberry Pi camera interface
import time                           # timing functions
import pigpio                         # hardware-timed pulses
import RPi.GPIO as GPIO               # GPIO control

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

# ——— Calibration ———
STEPS_PER_UNIT = 1  # conversion factor: units (e.g., pixels or mm) → step counts

# ——— Setup ———
GPIO.setmode(GPIO.BCM)  # use BCM numbering

# direction & step pins
for pin in (DirPin_1, DirPin_2, DirPin_3,
            StepPin_1, StepPin_2, StepPin_3):
    GPIO.setup(pin, GPIO.OUT)

# limit switches & belt stop input
GPIO.setup(Belt_Stopped, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
for sw in (LimitSwitch_M1, LimitSwitch_M2, LimitSwitch_M3):
    GPIO.setup(sw, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# ——— Stepper control class ———
class StepperSystem:
    def __init__(self):
        self.pi = pigpio.pi()
        # axis mapping: {'x':{'step':..., 'dir':...}, ...}
        self.axes = {
            'x': {'step': StepPin_1, 'dir': DirPin_1},
            'y': {'step': StepPin_2, 'dir': DirPin_2},
            'z': {'step': StepPin_3, 'dir': DirPin_3},
        }
        self.limits = {
            'x': LimitSwitch_M1,
            'y': LimitSwitch_M2,
            'z': LimitSwitch_M3,
        }
        # single camera instance
        self.picam = Picamera2()
        self.picam.start()

    # ——— Piecewise‐concurrent stepping (waveform) ———
    def step_parallel(self, steps, pulse_us=Pulse_width_us):
        """
        Move axes in parallel via pigpio waveform.
        steps: dict {'x':int, 'y':int, 'z':int}
        Positive → CW_Direction, negative → CCW_Direction.
        """
        # 1) set direction pins
        for ax, cnt in steps.items():
            d = CW_Direction if cnt >= 0 else CCW_Direction
            self.pi.write(self.axes[ax]['dir'], d)

        # 2) prepare Bresenham-style stepping
        total = max(abs(c) for c in steps.values())
        if total == 0:
            return
        counts = {ax: abs(c) for ax, c in steps.items()}
        err = {ax: 0 for ax in steps}
        pulses = []

        # 3) generate waveform pulses
        for _ in range(total):
            for ax in steps:
                err[ax] += counts[ax]
                if err[ax] >= total:
                    pin = self.axes[ax]['step']
                    pulses.append(pigpio.pulse(1<<pin, 0, pulse_us))
                    pulses.append(pigpio.pulse(0, 1<<pin, pulse_us))
                    err[ax] -= total

        # 4) send waveform once
        self.pi.wave_add_generic(pulses)
        wid = self.pi.wave_create()
        self.pi.wave_send_once(wid)
        while self.pi.wave_tx_busy():
            pass
        self.pi.wave_delete(wid)

    # ——— Trapezoidal-velocity profile for single axis ———
    def step_trapezoidal(self, axis, steps, v_max, accel, pulse_us=Pulse_width_us):
        """
        Move one axis with a trapezoidal velocity profile.
        axis: 'x', 'y', or 'z'
        steps: signed int (positive → CW, negative → CCW)
        v_max: max speed in steps/sec
        accel: acceleration in steps/sec^2
        """
        cnt = abs(steps)
        if cnt == 0:
            return
        # set direction
        d = CW_Direction if steps >= 0 else CCW_Direction
        self.pi.write(self.axes[axis]['dir'], d)
        # distance for accel phase
        d_a = v_max**2 / (2 * accel)
        if cnt < 2 * d_a:
            # triangular profile
            d_a = cnt / 2.0
            v_peak = (accel * cnt)**0.5
        else:
            v_peak = v_max
        n_a = int(d_a)
        n_c = cnt - 2 * n_a
        # build dt list
        dt_list = []
        # accel
        for i in range(1, n_a+1):
            v_i = (2 * accel * i)**0.5
            dt_list.append(1.0 / v_i)
        # cruise
        for _ in range(n_c):
            dt_list.append(1.0 / v_peak)
        # decel
        for i in range(n_a, 0, -1):
            v_i = (2 * accel * i)**0.5
            dt_list.append(1.0 / v_i)
        # generate pulses
        pulses = []
        pin = self.axes[axis]['step']
        for dt in dt_list:
            dt_us = int(dt * 1e6)
            low_delay = max(dt_us - pulse_us, 0)
            pulses.append(pigpio.pulse(1<<pin, 0, pulse_us))
            pulses.append(pigpio.pulse(0, 1<<pin, low_delay))
        self.pi.wave_add_generic(pulses)
        wid = self.pi.wave_create()
        self.pi.wave_send_once(wid)
        while self.pi.wave_tx_busy(): pass
        self.pi.wave_delete(wid)

    # ——— Homing routine ———
    def home(self):
        """
        Drive each axis toward its limit switch until all are triggered.
        Returns True when homed.
        """
        # set all to CCW direction
        for ax in self.axes:
            self.pi.write(self.axes[ax]['dir'], CCW_Direction)
        homed = {ax: False for ax in self.axes}

        # step until each limit switch closes
        while not all(homed.values()):
            for ax in self.axes:
                if not homed[ax]:
                    # one step at a time
                    self.step_parallel({ax: -1})
                    if GPIO.input(self.limits[ax]):
                        homed[ax] = True
        return True

    # ——— Z‑axis pickup action ———
    def pickup(self, steps=100):
        """
        Lower Z to pick, then raise Z. Uses fixed PickupSteps3.
        """
        # lower Z
        self.step_parallel({'z': -steps})
        # raise Z
        self.step_parallel({'z':  steps})

    # ——— Place corks at detected positions ———
    def place(self, results):
        """
        Move X & Y to each detected hole, then perform pickup.
        results: iterable of (distance, dx, dy) tuples.
        """
        for _, dx, dy in results:
            steps = {
                'x': int(dx * STEPS_PER_UNIT),
                'y': int(dy * STEPS_PER_UNIT),
                'z': 0
            }
            # move in X & Y together
            self.step_parallel(steps)
            # drop cork and retract
            self.pickup()

    # ——— Vision: detect circles ———
    def capture_and_detect(self, retries=5, delay=0.5):
        """
        Capture an image, threshold, find four circles via Hough.
        Return four (distance, dx, dy) tuples in sorted order.
        """
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
        blk = cv2.adaptiveThreshold(gray, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 51, -2)

        circles = cv2.HoughCircles(
            blk, cv2.HOUGH_GRADIENT, 1, 75,
            param1=110, param2=13,
            minRadius=25, maxRadius=37
        )
        if circles is None or len(circles[0]) < 4:
            return None

        pts = sorted(
            [(c[0], c[1]) for c in np.uint16(np.around(circles))[0,:4]],
            key=lambda p: (p[1], p[0])
        )
        def dist(a, b): return np.linalg.norm(np.array(a)-np.array(b))
        res = []
        for i in range(4):
            a, b = pts[i], pts[(i+1)%4]
            d = dist(a, b)
            res.append((d, b[0]-a[0], b[1]-a[1]))
        return tuple(res)

    # ——— Main control loop ———
    def main(self):
        homed = False
        while True:
            if not homed:
                homed = self.home()
            else:
                if GPIO.input(Belt_Stopped):
                    results = self.capture_and_detect()
                    if results:
                        self.place(results)
                        back_x = -sum(r[1] for r in results)
                        back_y = -sum(r[2] for r in results)
                        self.step_parallel({'x': back_x, 'y': back_y, 'z':0})
            time.sleep(0.1)  # prevent CPU hogging

# ——— Entry point ———
if __name__ == "__main__":
    system = StepperSystem()
    system.main()
