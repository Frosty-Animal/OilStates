import cv2
import numpy as np
from picamera2 import Picamera2
import time
import pigpio
import RPi.GPIO as GPIO


LimitSwitch_M1 = 25 # Pin Number
LimitSwitch_M2 = 24 # Pin Number
LimitSwitch_M3 = 23 # Pin Number
Homed = False
OperationComplete =  False
StepPin_1 = 18 # Pin Number
StepPin_2 = 12 # Pin Number
StepPin_3 =  20 # Pin Number
DirPin_1 = 19 # Pin Number 
DirPin_2 = 13 # Pin Number
DirPin_3 = 21 # Pin Number
CW_Direction = 0
CCW_Direction = 1
Circle_Positions = (dist1,dist2,dist3,dist4)
Pulse_width = .000003 # PWM speed
Belt_Stopped = 37 # Pin Number

GPIO.setmode(GPIO.BCM)
GPIO.setup(DirPin_1, GPIO.OUT)
GPIO.setup(StepPin_1, GPIO.OUT)
GPIO.setup(DirPin_2, GPIO.OUT)
GPIO.setup(StepPin_2, GPIO.OUT)
GPIO.setup(DirPin_3, GPIO.OUT)
GPIO.setup(StepPin_3, GPIO.OUT)

def main():
    while(true):
        
        # Step 1: Home motors
        if(homed == False):
            home([LimitSwitch_M1, LimitSwitch_M2, LimitSwitch_M3])
            homed = True
        else:
            # Step 2: Await belt to stop and get coordinates of holes
            if(Belt_Stopped == True):
                results = capture_and_detect()
                print(results)
            # Step 3: Use coordinates to Pickup and Place corks
            # Step 4: Check to make sure Task is complete from this and glueing station
            # Reapeat

def virtual_pwm():
    # use PigPio Library for virtual pwm implementation.

def home(Homed):
    
    if(LimitSwitch_M1 == 1 & LimitSwitch_M2 == 1 & LimitSwitch_M3 == 1):
        Homed = True
        return Homed
    # Move motors untill limit switch's are hit

def Pickup():
    
    # This will always be a fixed value in order to pickup cork
    if(task_completed == False):
        for _ in range(steps):
            GPIO.output(StepPin_1, GPIO.HIGH)
            time.sleep(Pulse_width)  
            GPIO.output(StepPin_1, GPIO.LOW)
            time.sleep(Pulse_width)
    else:
        return

def Place():
    # Take in coordinates to move motors to place the corks. Z axis will be fixed values.
    return

def capture_and_detect():
    
    
    # Initialize the camera
    try:
        picam2 = Picamera2()
        picam2.start()
        frame = picam2.capture_array()
        picam2.close()
    except RuntimeError as e:
        print(f"Camera error: {e}")
        time.sleep(0.5)
        return capture_and_detect()

    # Convert the image to BGR format for OpenCV
    img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # Get image dimensions
    img_height, img_width = img.shape[:2]

    # Convert to grayscale and apply thresholding
    blk = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blk = cv2.medianBlur(blk, 11)
    blk = cv2.adaptiveThreshold(blk, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 51, -2)

    # Find circles using HoughCircles
    circles = cv2.HoughCircles(blk, cv2.HOUGH_GRADIENT, 1, 75, param1=110, param2=13, minRadius=25, maxRadius=37)

    if circles is not None and len(circles[0]) >= 4:
        circles = np.uint16(np.around(circles))
        print(f"{len(circles[0])} circles detected!")

        # Collect circle positions
        circle_positions = [(circle[0], circle[1]) for circle in circles[0, :4]]

        # Sort the circles for consistent ordering
        circle_positions = sorted(circle_positions, key=lambda p: (p[1], p[0]))

        # Extract individual positions
        c1, c2, c3, c4 = circle_positions

        cv2.circle(img, (int(c1[0]), int(c1[1])), 5, (0, 255, 0), -1)
        cv2.circle(img, (int(c2[0]), int(c2[1])), 5, (0, 255, 0), -1)
        cv2.circle(img, (int(c3[0]), int(c3[1])), 5, (0, 255, 0), -1)
        cv2.circle(img, (int(c4[0]), int(c4[1])), 5, (0, 255, 0), -1)

        cv2.line(img, (int(c1[0]), int(c1[1])), (int(c2[0]), int(c2[1])), (255, 0, 0), 2)
        cv2.line(img, (int(c2[0]), int(c2[1])), (int(c3[0]), int(c3[1])), (255, 0, 0), 2)
        cv2.line(img, (int(c3[0]), int(c3[1])), (int(c4[0]), int(c4[1])), (255, 0, 0), 2)
        cv2.line(img, (int(c4[0]), int(c4[1])), (int(c1[0]), int(c1[1])), (255, 0, 0), 2)
        
        # Calculate total distances
        dist1 = np.linalg.norm(np.array(c1) - np.array(c2))  
        dist2 = np.linalg.norm(np.array(c2) - np.array(c3)) 
        dist3 = np.linalg.norm(np.array(c3) - np.array(c4))
        dist4 = np.linalg.norm(np.array(c4) - np.array(c1))

        # Calculate x and y differences
        dx1, dy1 = c2[0] - c1[0], c2[1] - c1[1]
        dx2, dy2 = c3[0] - c2[0], c3[1] - c2[1]
        dx3, dy3 = c4[0] - c3[0], c4[1] - c3[1]
        dx4, dy4 = c1[0] - c4[0], c1[1] - c4[1]
        print(f"Dist1: {dist1:.2f}, Dist2: {dist2:.2f}, Dist3: {dist3:.2f}, Dist4: {dist4:.2f}")
        
        #Display the distances on the image
        cv2.putText(img, f"{dist1:.2f}", (int((c1[0] + c2[0]) / 2 + 20), int((c1[1] + c2[1]) / 2 + 20)), cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 255, 0))
        cv2.putText(img, f"{dist2:.2f}", (int((c2[0] + c3[0]) / 2 + 20), int((c2[1] + c3[1]) / 2 + 20)), cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 0, 255))
        cv2.putText(img, f"{dist3:.2f}", (int((c3[0] + c4[0]) / 2 + 20), int((c3[1] + c4[1]) / 2 + 20)), cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 255, 255))
        cv2.putText(img, f"{dist4:.2f}", (int((c4[0] + c1[0]) / 2 + 20), int((c4[1] + c1[1]) / 2 + 20)), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0, 255, 255.))
        
        # Return all distances and x, y differences
        return (dist1, dx1, dy1), (dist2, dx2, dy2), (dist3, dx3, dy3), (dist4, dx4, dy4)
