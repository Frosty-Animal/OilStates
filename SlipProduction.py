import cv2
import numpy as np
from picamera2 import Picamera2
import time
import pigpio
import RPi.GPIO as GPIO
import random
import os

# ——— Pin Definitions ———
LimitSwitch_M1 = 25
LimitSwitch_M2 = 24
LimitSwitch_M3 = 23
Belt_Stopped    = 37

StepPin_1 = 18
StepPin_2 = 12
StepPin_3 = 20
DirPin_1  = 19
DirPin_2  = 13
DirPin_3  = 21

CW_Direction  = 0
CCW_Direction = 1
Pulse_width   = .000003  # PWM speed

# ——— Calibration ———
# TODO: set this to your mm (or pixel) → step count conversion
STEPS_PER_UNIT = 1  

# ——— GPIO Setup ———
GPIO.setmode(GPIO.BCM)
for pin in (DirPin_1, DirPin_2, DirPin_3, StepPin_1, StepPin_2, StepPin_3):
    GPIO.setup(pin, GPIO.OUT)

# ——— Piecewise‐concurrent step function ———
def step_parallel(steps1, steps2, steps3):
    # set direction for each
    dirs = (
        (DirPin_1, steps1),
        (DirPin_2, steps2),
        (DirPin_3, steps3),
    )
    for dir_pin, s in dirs:
        GPIO.output(dir_pin, CW_Direction if s >= 0 else CCW_Direction)

    # interleave pulses
    total = max(abs(steps1), abs(steps2), abs(steps3))
    for i in range(total):
        if i < abs(steps1): GPIO.output(StepPin_1, GPIO.HIGH)
        if i < abs(steps2): GPIO.output(StepPin_2, GPIO.HIGH)
        if i < abs(steps3): GPIO.output(StepPin_3, GPIO.HIGH)

        time.sleep(Pulse_width)

        GPIO.output(StepPin_1, GPIO.LOW)
        GPIO.output(StepPin_2, GPIO.LOW)
        GPIO.output(StepPin_3, GPIO.LOW)

        time.sleep(Pulse_width)

# ——— Homing routine ———
def home():
    GPIO.output(DirPin_1, CCW_Direction)
    GPIO.output(DirPin_2, CCW_Direction)
    GPIO.output(DirPin_3, CCW_Direction)

    homed = [False, False, False]
    while not all(homed):
        if not homed[0]:
            GPIO.output(StepPin_1, GPIO.HIGH); time.sleep(Pulse_width)
            GPIO.output(StepPin_1, GPIO.LOW);  time.sleep(Pulse_width)
            if GPIO.input(LimitSwitch_M1): homed[0] = True

        if not homed[1]:
            GPIO.output(StepPin_2, GPIO.HIGH); time.sleep(Pulse_width)
            GPIO.output(StepPin_2, GPIO.LOW);  time.sleep(Pulse_width)
            if GPIO.input(LimitSwitch_M2): homed[1] = True

        if not homed[2]:
            GPIO.output(StepPin_3, GPIO.HIGH); time.sleep(Pulse_width)
            GPIO.output(StepPin_3, GPIO.LOW);  time.sleep(Pulse_width)
            if GPIO.input(LimitSwitch_M3): homed[2] = True

    return True

# ——— Pickup Z (down/up) ———
def Pickup():
    # move Z down then up
    GPIO.output(DirPin_3, CCW_Direction)
    step_parallel(0, 0, PickupSteps3 := 100)  # adjust steps
    GPIO.output(DirPin_3, CW_Direction)
    step_parallel(0, 0, PickupSteps3)

# ——— Place at detected holes ———
def Place(results):
    # results is tuple of four (dist, dx, dy)
    for (_, dx, dy) in results:
        # convert to steps
        steps1 = int(dx * STEPS_PER_UNIT)
        steps2 = int(dy * STEPS_PER_UNIT)
        # move X & Y in parallel
        step_parallel(steps1, steps2, 0)
        # drop & retract Z
        Pickup()

# ——— Computer vision ———
def capture_and_detect():
    try:
        picam2 = Picamera2()
        picam2.start()
        frame = picam2.capture_array()
        picam2.close()
    except RuntimeError:
        time.sleep(0.5)
        return capture_and_detect()

    img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    gray = cv2.medianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 11)
    blk  = cv2.adaptiveThreshold(gray, 255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY, 51, -2)

    circles = cv2.HoughCircles(blk, cv2.HOUGH_GRADIENT, 1, 75,
                param1=110, param2=13,
                minRadius=25, maxRadius=37)
    if circles is None or len(circles[0]) < 4:
        return None

    circles = np.uint16(np.around(circles))[0,:4]
    pts = sorted([(c[0],c[1]) for c in circles], key=lambda p: (p[1],p[0]))
    (c1,c2,c3,c4) = pts
    # compute distances & deltas
    def dist(a,b): return np.linalg.norm(np.array(a)-np.array(b))
    d1,dx1,dy1 = dist(c1,c2), c2[0]-c1[0], c2[1]-c1[1]
    d2,dx2,dy2 = dist(c2,c3), c3[0]-c2[0], c3[1]-c2[1]
    d3,dx3,dy3 = dist(c3,c4), c4[0]-c3[0], c4[1]-c3[1]
    d4,dx4,dy4 = dist(c4,c1), c1[0]-c4[0], c1[1]-c4[1]

    return ((d1,dx1,dy1),
            (d2,dx2,dy2),
            (d3,dx3,dy3),
            (d4,dx4,dy4))

# ——— Main loop ———
def main():
    Homed = False
    while True:
        if not Homed:
            Homed = home()
        else:
            if GPIO.input(Belt_Stopped):
                results = capture_and_detect()
                if results:
                    Place(results)
                    # return home after placing all 4
                    step_parallel(-sum(r[1] for r in results),
                                   -sum(r[2] for r in results), 0)
        time.sleep(0.1)

if __name__ == "__main__":
    main()
