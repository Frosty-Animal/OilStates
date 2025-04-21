import cv2                            # OpenCV for image processing
import numpy as np                    # numerical operations
from picamera2 import Picamera2      # Raspberry Pi camera interface
import time                           # timing functions
import pigpio                         # hardware PWM (unused here)
import RPi.GPIO as GPIO               # GPIO control
import random                         # for random numbers (not used here)
import os                             # operating system utilities (not used here)

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
Pulse_width   = 0.000003   # delay between step high/low for speed control

# ——— Calibration ———
STEPS_PER_UNIT = 1  # conversion factor: units (e.g., pixels or mm) → step counts

# ——— GPIO Setup ———
GPIO.setmode(GPIO.BCM)  # use BCM numbering
for pin in (DirPin_1, DirPin_2, DirPin_3, StepPin_1, StepPin_2, StepPin_3):
    GPIO.setup(pin, GPIO.OUT)  # configure each control pin as output

# ——— Piecewise‐concurrent stepping ———
def step_parallel(steps1, steps2, steps3):
    """
    Move up to three axes “in parallel” by interleaving pulses.
    stepsN: signed integer number of steps for each motor.
    Positive → CW_Direction, negative → CCW_Direction.
    """
    # 1) set direction pins based on sign of each step count
    for dir_pin, count in ((DirPin_1, steps1), (DirPin_2, steps2), (DirPin_3, steps3)):
        GPIO.output(dir_pin, CW_Direction if count >= 0 else CCW_Direction)

    # 2) issue step pulses up to the longest required count
    total = max(abs(steps1), abs(steps2), abs(steps3))
    for i in range(total):
        # raise each STEP pin if more pulses remain for that axis
        if i < abs(steps1): GPIO.output(StepPin_1, GPIO.HIGH)
        if i < abs(steps2): GPIO.output(StepPin_2, GPIO.HIGH)
        if i < abs(steps3): GPIO.output(StepPin_3, GPIO.HIGH)

        time.sleep(Pulse_width)  # high‑pulse duration

        # lower all STEP pins
        GPIO.output(StepPin_1, GPIO.LOW)
        GPIO.output(StepPin_2, GPIO.LOW)
        GPIO.output(StepPin_3, GPIO.LOW)

        time.sleep(Pulse_width)  # low‑pulse duration

# ——— Homing routine ———
def home():
    """
    Drive each axis toward its limit switch until all are triggered.
    Returns True when homed.
    """
    # 1) set all axes to move in negative (CCW) direction
    GPIO.output(DirPin_1, CCW_Direction)
    GPIO.output(DirPin_2, CCW_Direction)
    GPIO.output(DirPin_3, CCW_Direction)

    homed = [False, False, False]  # track each axis
    # 2) step each axis individually until its switch closes
    while not all(homed):
        # X‑axis
        if not homed[0]:
            GPIO.output(StepPin_1, GPIO.HIGH); time.sleep(Pulse_width)
            GPIO.output(StepPin_1, GPIO.LOW);  time.sleep(Pulse_width)
            if GPIO.input(LimitSwitch_M1):
                homed[0] = True
        # Y‑axis
        if not homed[1]:
            GPIO.output(StepPin_2, GPIO.HIGH); time.sleep(Pulse_width)
            GPIO.output(StepPin_2, GPIO.LOW);  time.sleep(Pulse_width)
            if GPIO.input(LimitSwitch_M2):
                homed[1] = True
        # Z‑axis
        if not homed[2]:
            GPIO.output(StepPin_3, GPIO.HIGH); time.sleep(Pulse_width)
            GPIO.output(StepPin_3, GPIO.LOW);  time.sleep(Pulse_width)
            if GPIO.input(LimitSwitch_M3):
                homed[2] = True

    return True  # now homed

# ——— Z‑axis pickup action ———
def Pickup():
    """
    Lower Z to pick (move down), then raise Z (move up).
    Uses a fixed number of steps (tune PickupSteps3).
    """
    PickupSteps3 = 100  # tune for your hardware
    # lower Z
    GPIO.output(DirPin_3, CCW_Direction)
    step_parallel(0, 0, PickupSteps3)
    # raise Z
    GPIO.output(DirPin_3, CW_Direction)
    step_parallel(0, 0, PickupSteps3)

# ——— Place corks at detected positions ———
def Place(results):
    """
    Move X & Y to each detected hole, then perform Pickup.
    results: iterable of (distance, dx, dy) tuples.
    """
    for (_, dx, dy) in results:
        # convert image‑space deltas into step counts
        steps1 = int(dx * STEPS_PER_UNIT)
        steps2 = int(dy * STEPS_PER_UNIT)
        # move in X & Y together
        step_parallel(steps1, steps2, 0)
        # drop cork and retract (Z movement)
        Pickup()

# ——— Vision: detect four circles (holes) ———
def capture_and_detect():
    """
    Capture an image, threshold, find circles via Hough, return
    four (distance, dx, dy) tuples in sorted order.
    """
    try:
        picam2 = Picamera2()
        picam2.start()
        frame = picam2.capture_array()
        picam2.close()
    except RuntimeError:
        time.sleep(0.5)
        return capture_and_detect()  # retry on camera error

    # prepare binary image for circle detection
    img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    gray = cv2.medianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 11)
    blk  = cv2.adaptiveThreshold(gray, 255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY, 51, -2)

    # detect circles
    circles = cv2.HoughCircles(
        blk, cv2.HOUGH_GRADIENT, 1, 75,
        param1=110, param2=13,
        minRadius=25, maxRadius=37
    )
    if circles is None or len(circles[0]) < 4:
        return None  # not enough circles

    # take first 4 circles, sort by Y then X for consistent order
    pts = sorted(
        [(c[0], c[1]) for c in np.uint16(np.around(circles))[0,:4]],
        key=lambda p: (p[1], p[0])
    )
    c1, c2, c3, c4 = pts

    # helper to compute Euclidean distance
    def dist(a, b): return np.linalg.norm(np.array(a) - np.array(b))

    # compute distance + delta for each edge
    d1, dx1, dy1 = dist(c1, c2), c2[0]-c1[0], c2[1]-c1[1]
    d2, dx2, dy2 = dist(c2, c3), c3[0]-c2[0], c3[1]-c2[1]
    d3, dx3, dy3 = dist(c3, c4), c4[0]-c3[0], c4[1]-c3[1]
    d4, dx4, dy4 = dist(c4, c1), c1[0]-c4[0], c1[1]-c4[1]

    return ((d1, dx1, dy1),
            (d2, dx2, dy2),
            (d3, dx3, dy3),
            (d4, dx4, dy4))

# ——— Main control loop ———
def main():
    Homed = False
    while True:
        if not Homed:
            # perform homing on startup
            Homed = home()
        else:
            # once homed, wait for belt to stop
            if GPIO.input(Belt_Stopped):
                results = capture_and_detect()
                if results:
                    Place(results)
                    # optional: return X/Y to start
                    back_x = -sum(r[1] for r in results)
                    back_y = -sum(r[2] for r in results)
                    step_parallel(back_x, back_y, 0)
        time.sleep(0.1)  # prevent CPU hogging

if __name__ == "__main__":
    main()
