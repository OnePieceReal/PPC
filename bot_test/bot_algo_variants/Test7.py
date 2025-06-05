#!/usr/bin/env python3
"""
Integrated Robot Control with Priority and Stuck Timer:
  1. Obstacle Avoidance (always active)
  2. Ping-Pong Ball Collection
  3. Continuous Autofocus

Behavior:
   The LiDAR thread continuously scans and updates global variables (avoidance_active and avoidance_maneuver)
    based on obstacles detected in a defined front sector.
   The detection thread uses a camera and Jetson Inference to detect ping-pong balls ("orange_pp")
    and updates a global ball_target.
   The autofocus thread continuously runs an external autofocus script ("Focuser.py").
   The main control loop always gives obstacle avoidance the highest priority.
       - If an obstacle is detected, the robot executes the avoidance maneuver.
       - If no obstacle is detected, it checks for a ball.
             * If a ball is detected, it enters a collection phase.
             * A stuck timer monitors if the same turning command repeats too long, and if so, overrides it with a forward drive.
       - Once the ball is collected or lost, control immediately reverts to obstacle avoidance.
   On termination (via KeyboardInterrupt), all threads are stopped and the LiDAR motor is stopped/disconnected.
  
Ensure that the LiDAR and Arduino use separate serial ports.
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

# Serial communication settings (ensure LiDAR and Arduino are on separate ports)
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
    print("Pivoting (left pivot)")

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

# ----------------------- LiDAR Obstacle Avoidance Thread -----------------------

def lidar_thread():
    global running, avoidance_active, avoidance_maneuver
    print("LiDAR thread starting...")
    # Angle thresholds and distance threshold (tweak as needed)
    FRONT_START = 40    # degrees
    FRONT_END   = 140   # degrees
    DIST_THRESHOLD = 465  # mm

    try:
        lidar = RPLidar(LIDAR_PORT)
        print("LiDAR initialized on port:", LIDAR_PORT)
    except Exception as e:
        print("Failed to initialize LiDAR:", e)
        return

    try:
        for scan in lidar.iter_scans():
            if not running:
                break

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

            print("LiDAR scan points:", len(scan))
            print("Front distances:", front_dists)
            print("Left distances:", left_dists)
            print("Right distances:", right_dists)
            with avoidance_lock:
                print("Avoidance active:", avoidance_active, "Maneuver:", avoidance_maneuver)
            time.sleep(0.1)
    except Exception as e:
        print("LiDAR thread exception:", e)
    finally:
        try:
            print("Stopping LiDAR motor...")
            lidar.stop_motor()
        except Exception as e:
            print("Error stopping LiDAR motor:", e)
        try:
            print("Disconnecting LiDAR...")
            lidar.disconnect()
        except Exception as e:
            print("Error disconnecting LiDAR:", e)
        print("LiDAR thread terminated.")

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
        print("Autofocus thread: running autofocus routine...")
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

# ----------------------- Proportional Turn Function -----------------------

def proportional_turn(error):
    # Gain constant determines how strongly the robot turns in response to error.
    k = 0.05  # Adjust this value as needed
    turn_rate = k * error
    base_speed = 50  # This is your nominal forward speed (0-100 scale, for example)

    # Calculate left and right speeds based on the error.
    left_speed = base_speed + turn_rate
    right_speed = base_speed - turn_rate

    # Clamp speeds between 0 and 100.
    left_speed = max(0, min(100, left_speed))
    right_speed = max(0, min(100, right_speed))

    print(f"Proportional turn: error={error:.2f}, left_speed={left_speed:.2f}, right_speed={right_speed:.2f}")

    # If PWM is not available, use a simple fallback:
    if error > 0:
        right()  # Turn right if the ball is to the right.
    else:
        left()   # Turn left if the ball is to the left.

# ----------------------- Main Control Loop (with Proportional Controller) -----------------------

def main_control_loop():
    global running, ball_target
    print("Starting main control loop...")
    stuck_timer = 0.0
    last_cmd = None  # Store the last command issued

    while running:
        # Priority 1: Obstacle avoidance always takes precedence.
        with avoidance_lock:
            obstacle = avoidance_active
            maneuver = avoidance_maneuver

        # Determine current command
        current_cmd = None

        if obstacle:
            print("State: Obstacle Avoidance")
            if maneuver == "left":
                current_cmd = "left"
            elif maneuver == "right":
                current_cmd = "right"
            else:
                # When obstacle readings are ambiguous, use pivot maneuver.
                current_cmd = "pivot"
        else:
            # Priority 2: Check for ball detection
            with ball_target_lock:
                current_ball = ball_target.copy() if ball_target is not None else None

            if current_ball is not None:
                print("State: Object Detected")
                cx = current_ball["center"][0]
                top_val = current_ball["top"]
                print(f"Pursuing ball at X={cx}, Top={top_val}")
                
                # Proportional control implementation:
                desired_center = 650  # Adjust as needed based on your camera resolution
                tolerance = 30       # Deadband tolerance in pixels
                error = cx - desired_center

                if abs(error) < tolerance:
                    # Ball is sufficiently centered; if close enough vertically, collect it.
                    if top_val > 600:
                        current_cmd = "collect"
                    else:
                        current_cmd = "forward"
                else:
                    # Use a proportional turning command.
                    current_cmd = ("turn", error)
            else:
                print("State: Roaming")
                current_cmd = "forward"

        # Update stuck timer (if current_cmd is a tuple, use its first element)
        cmd_str = current_cmd if isinstance(current_cmd, str) else current_cmd[0]
        if cmd_str == last_cmd:
            stuck_timer += 0.1
        else:
            stuck_timer = 0.0
            last_cmd = cmd_str

        # Override if stuck
        if stuck_timer > 0.5:
            print("Stuck command detected; overriding with forward drive.")
            forward()
            stuck_timer = 0.0
            last_cmd = "forward"
        else:
            # Execute the current command.
            if isinstance(current_cmd, tuple) and current_cmd[0] == "turn":
                proportional_turn(current_cmd[1])
            elif current_cmd == "left":
                left()
            elif current_cmd == "right":
                right()
            elif current_cmd == "pivot":
                leftPivot()
            elif current_cmd == "collect":
                kill()
                # Collection logic remains unchanged.
                with ball_target_lock:
                    cx = current_ball["center"][0]
                if cx > 900:
                    right()
                elif cx < 420:
                    left()
                else:
                    print("Ball centered  entering collection phase")
                    collection_duration = 1.5  # seconds; adjust as needed
                    sendUART(1)  # Start collection command
                    start_time = time.time()
                    while time.time() - start_time < collection_duration:
                        forward()
                        time.sleep(0.1)
                    sendUART(3)  # Return to normal mode after collection
                    with ball_target_lock:
                        ball_target = None
            else:
                forward()

        time.sleep(0.1)

# ----------------------- Main -----------------------

def main():
    global running
    print("Initializing resources...")
    setup_gpio()

    # Start the LiDAR, detection, and autofocus threads.
    lidar_thread_obj = threading.Thread(target=lidar_thread)
    detection_thread_obj = threading.Thread(target=detection_thread)
    autofocus_thread_obj = threading.Thread(target=autofocus_thread)

    lidar_thread_obj.start()
    detection_thread_obj.start()
    autofocus_thread_obj.start()

    # Allow sensors to warm up.
    time.sleep(3)
    print("Initialization complete. Starting motion...")

    try:
        main_control_loop()
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Stopping...")
        running = False

    lidar_thread_obj.join()
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
