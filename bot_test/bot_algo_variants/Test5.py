#!/usr/bin/env python3
"""
State-Machine Based Integrated Robot Control with Bounded Queue for LiDAR

Behavior:
   A LiDAR Reader Thread continuously reads scans from the LiDAR and places them into a 
    bounded queue (size=1) so that only the most recent scan is processed.
   A LiDAR Processing Thread pulls scans from the queue, calculates obstacle information, 
    and updates global variables (avoidance_active and avoidance_maneuver).
   The detection thread uses a camera and Jetson Inference to detect ping-pong balls ("orange_pp")
    and updates a global ball_target.
   The autofocus thread continuously runs an external autofocus script ("Focuser.py").
   The main control loop first checks obstacle avoidance (highest priority). If an obstacle is detected, 
    it executes the avoidance maneuver. Only if no obstacle is present does it check for a ball target 
    and run the collection routine. Once the ball is collected or lost, it immediately reverts to obstacle avoidance.
   On termination (KeyboardInterrupt), all threads are stopped and the LiDAR motor is stopped and disconnected.
  
Make sure that LiDAR and Arduino use separate serial ports.
"""

import time
import threading
import subprocess
import serial
import math
import numpy as np
import queue
import RPi.GPIO as GPIO
from rplidar import RPLidar
from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput

# ----------------------- Global Definitions -----------------------

# GPIO pin definitions (adjust as needed)
ENA, IN1, IN2 = 33, 37, 35
ENB, IN3, IN4 = 32, 40, 38

running = True  # Global run flag

# Global obstacle avoidance variables (updated by LiDAR processing thread)
avoidance_active = False       # True if an obstacle is detected
avoidance_maneuver = None      # "left", "right", or "pivot"
avoidance_lock = threading.Lock()  # Protects these variables

# Global ball target (updated by detection thread)
ball_target = None
ball_target_lock = threading.Lock()

# Bounded queue for LiDAR scans (only keep the most recent scan)
lidar_queue = queue.Queue(maxsize=1)

# Serial communication settings (ensure LiDAR and Arduino use separate ports)
arduino = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
LIDAR_PORT = "/dev/ttyUSB0"

# ----------------------- Motor Control Functions -----------------------

def forward():
    GPIO.output(ENA, GPIO.HIGH)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(ENB, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    print("Moving forward")

def left():
    GPIO.output(ENA, GPIO.LOW)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(ENB, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    print("Turning left")

def right():
    GPIO.output(ENA, GPIO.HIGH)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(ENB, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    print("Turning right")

def leftPivot():
    GPIO.output(ENA, GPIO.HIGH)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(ENB, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    print("Left pivot")

def kill():
    GPIO.output(ENA, GPIO.LOW)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(ENB, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    print("Motors stopped")

def sendUART(ardcom):
    ardcomChar = str(ardcom)
    print(f"Sending UART command: {ardcomChar}")
    arduino.write(ardcomChar.encode())
    arduino.flush()

# ----------------------- GPIO Setup -----------------------

def setup_gpio():
    GPIO.setmode(GPIO.BOARD)
    for pin in [ENA, IN1, IN2, ENB, IN3, IN4]:
        GPIO.setup(pin, GPIO.OUT)

# ----------------------- LiDAR Reader Thread -----------------------

def lidar_reader_thread():
    global running, lidar_queue
    try:
        lidar = RPLidar(LIDAR_PORT)
        print("LiDAR Reader: Initialized on port", LIDAR_PORT)
        # Uncomment if motor startup is needed:
        # lidar.start_motor()
        # time.sleep(3)
    except Exception as e:
        print("LiDAR Reader: Failed to initialize:", e)
        return

    try:
        for scan in lidar.iter_scans():
            if not running:
                break
            # Keep only the most recent scan in the queue.
            if not lidar_queue.empty():
                try:
                    lidar_queue.get_nowait()
                except queue.Empty:
                    pass
            lidar_queue.put(scan)
            time.sleep(0.1)
    except Exception as e:
        print("LiDAR Reader exception:", e)
    finally:
        try:
            print("LiDAR Reader: Stopping motor...")
            lidar.stop_motor()
        except Exception as e:
            print("LiDAR Reader: Error stopping motor:", e)
        try:
            print("LiDAR Reader: Disconnecting...")
            lidar.disconnect()
        except Exception as e:
            print("LiDAR Reader: Error disconnecting:", e)
        print("LiDAR Reader thread terminated.")

# ----------------------- LiDAR Processing Thread -----------------------

def lidar_processing_thread():
    global running, avoidance_active, avoidance_maneuver, lidar_queue
    # Define thresholds (adjust as needed)
    FRONT_START = 60    # degrees
    FRONT_END   = 120   # degrees
    DIST_THRESHOLD = 475  # mm

    while running:
        try:
            # Wait up to 1 second for a scan.
            scan = lidar_queue.get(timeout=1)
        except queue.Empty:
            continue

        front_dists = []
        left_dists = []
        right_dists = []
        for (_, angle, distance) in scan:
            if distance < DIST_THRESHOLD:
                if FRONT_START <= angle <= FRONT_END:
                    front_dists.append(distance)
                elif angle < FRONT_START:
                    left_dists.append(distance)
                elif angle > FRONT_END:
                    right_dists.append(distance)

        with avoidance_lock:
            if len(front_dists) == 0:
                avoidance_active = False
                avoidance_maneuver = None
            else:
                avoidance_active = True
                avg_left = sum(left_dists) / len(left_dists) if left_dists else float('inf')
                avg_right = sum(right_dists) / len(right_dists) if right_dists else float('inf')
                if avg_left > avg_right:
                    avoidance_maneuver = "left"
                elif avg_right > avg_left:
                    avoidance_maneuver = "right"
                else:
                    avoidance_maneuver = "pivot"
        # Debug prints
        print("LiDAR Process: Scan points:", len(scan))
        print("LiDAR Process: Front distances:", front_dists)
        print("LiDAR Process: Left distances:", left_dists)
        print("LiDAR Process: Right distances:", right_dists)
        with avoidance_lock:
            print("LiDAR Process: Avoidance active:", avoidance_active, "Maneuver:", avoidance_maneuver)
        time.sleep(0.1)

# ----------------------- Detection Thread -----------------------

def detection_thread():
    global running, ball_target
    net = detectNet(argv=[
        "--model=/home/capstone/Desktop/maincapstone/pingPongBallCollector/608Final/ssd-mobilenet.onnx",
        "--labels=/home/capstone/Desktop/maincapstone/pingPongBallCollector/608Final/labels.txt",
        "--input-blob=input_0",
        "--output-cvg=scores",
        "--output-bbox=boxes",
        "--threshold=0.35"
    ], threshold=0.40)
    camera = videoSource("csi://0")
    display = videoOutput("display://0")
    
    while running:
        img = camera.Capture()
        if img is None:
            time.sleep(0.05)
            continue

        detections = net.Detect(img)
        maxObj = None
        for obj in detections:
            if obj.ClassID == 1:  # "orange_pp"
                if maxObj is None or obj.Area > maxObj.Area:
                    maxObj = obj

        with ball_target_lock:
            if maxObj is not None:
                ball_target = {
                    "center": maxObj.Center,
                    "top": maxObj.Top,
                    "bottom": maxObj.Bottom,
                    "left": maxObj.Left,
                    "right": maxObj.Right
                }
        display.Render(img)
        display.SetStatus("Detection FPS: {:.0f}".format(net.GetNetworkFPS()))
        time.sleep(0.2)
    camera.Close()
    display.Close()

# ----------------------- Autofocus Thread -----------------------

def autofocus_thread():
    while running:
        print("Autofocus thread: running autofocus...")
        try:
            result = subprocess.run("python3 Focuser.py", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout = result.stdout.decode().strip()
            stderr = result.stderr.decode().strip()
            if stdout:
                print("Autofocus output:", stdout)
            if stderr:
                print("Autofocus error:", stderr)
        except Exception as e:
            print("Autofocus thread exception:", e)
        time.sleep(1)

# ----------------------- Main Control Loop (State Machine) -----------------------

def main_control_loop():
    global running, ball_target
    print("Starting main control loop...")
    while running:
        # Priority 1: Obstacle avoidance
        with avoidance_lock:
            obstacle = avoidance_active
            maneuver = avoidance_maneuver

        if obstacle:
            print("State: Obstacle Avoidance")
            if maneuver == "left":
                left()
            elif maneuver == "right":
                right()
            else:
                leftPivot()
        else:
            # Priority 2: Object detection & collection
            with ball_target_lock:
                current_ball = ball_target.copy() if ball_target is not None else None

            if current_ball is not None:
                print("State: Object Detected")
                cx = current_ball["center"][0]
                top_val = current_ball["top"]
                print(f"Pursuing ball at X={cx}, Top={top_val}")
                if top_val > 600:
                    kill()
                    if cx > 900:
                        right()
                    elif cx < 420:
                        left()
                    else:
                        print("Ball centered  entering collection phase")
                        collection_duration = 1.5  # seconds, adjust as needed
                        sendUART(1)  # Start collection command
                        start_time = time.time()
                        while time.time() - start_time < collection_duration:
                            forward()
                            time.sleep(0.1)
                        sendUART(3)  # Return to normal mode after collection
                        with ball_target_lock:
                            ball_target = None
                else:
                    if cx > 870:
                        right()
                    elif cx < 430:
                        left()
                    else:
                        forward()
            else:
                print("State: Roaming")
                forward()
        time.sleep(0.1)

# ----------------------- Main -----------------------

def main():
    global running
    print("Initializing resources...")
    setup_gpio()

    # Start LiDAR reader and processing threads.
    lidar_reader_obj = threading.Thread(target=lidar_reader_thread)
    lidar_processor_obj = threading.Thread(target=lidar_processing_thread)
    detection_thread_obj = threading.Thread(target=detection_thread)
    autofocus_thread_obj = threading.Thread(target=autofocus_thread)

    lidar_reader_obj.start()
    lidar_processor_obj.start()
    detection_thread_obj.start()
    autofocus_thread_obj.start()

    time.sleep(3)
    print("Initialization complete. Starting motion...")

    try:
        main_control_loop()
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Stopping...")
        running = False

    lidar_reader_obj.join()
    lidar_processor_obj.join()
    detection_thread_obj.join()
    autofocus_thread_obj.join()
    kill()
    GPIO.cleanup()
    sendUART(2)
    time.sleep(3)
    sendUART(0)
    print("Program terminated.")

if __name__ == "__main__":
    main()
