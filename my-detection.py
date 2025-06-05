#!/usr/bin/env python3
#
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#

from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput
import RPi.GPIO as GPIO
import time
import os
import serial

# Intialize PIN LAYOUT

ena = 33
in1 = 37
in2 = 35

enb = 32
in3 = 40
in4 = 38

# Intialize serial pin -> Serial.begin(9600) on arduino -> Serial.write(int) to arduino

arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# Initialize Model

#net = detectNet(argv=["--model=/home/capstone/Downloads/ppballs417images608resolution4batch200epoch/ssd-mobilenet.onnx", "--labels=/home/capstone/Downloads/ppballs417images512resolution20batch200epoch-20240329T061400Z-001/ppballs417images512resolution20batch200epoch/pytorch-ssd-res-512/models/ppballs350image512v2/labels.txt", "--input-blob=input_0", "--output-cvg=scores", "--output-bbox=boxes", "--threshold=0.25"], threshold=0.2)

#net = detectNet(argv=["--model=/home/capstone/Downloads/ppballs417images512resolution20batch200epoch-20240329T061400Z-001/ppballs417images512resolution20batch200epoch/pytorch-ssd-res-512/models/ppballs350image512v2/ssd-mobilenet.onnx", "--labels=/home/capstone/Downloads/ppballs417images512resolution20batch200epoch-20240329T061400Z-001/ppballs417images512resolution20batch200epoch/pytorch-ssd-res-512/models/ppballs350image512v2/labels.txt", "--input-blob=input_0", "--output-cvg=scores", "--output-bbox=boxes", "--threshold=0.25"], threshold=0.1)

#net = detectNet(argv=["--model=/home/capstone/Downloads/test300/ssd-mobilenet.onnx", "--labels=/home/capstone/Downloads/ppballs417images512resolution20batch200epoch-20240329T061400Z-001/ppballs417images512resolution20batch200epoch/pytorch-ssd-res-512/models/ppballs350image512v2/labels.txt", "--input-blob=input_0", "--output-cvg=scores", "--output-bbox=boxes", "--threshold=0.25"], threshold=0.1)

#net = detectNet(argv=["--model=/home/capstone/Desktop/maincapstone/pingPongBallCollector/608Final/ssd-mobilenet.onnx", "--labels=/home/capstone/Desktop/maincapstone/pingPongBallCollector/608Final/labels.txt", "--input-blob=input_0", "--output-cvg=scores", "--output-bbox=boxes", "--threshold=0.35"], threshold=0.15)

net = detectNet(argv=["--model=/home/capstone/Downloads/512Final/ssd-mobilenet.onnx", "--labels=/home/capstone/Downloads/512Final/labels.txt", "--input-blob=input_0", "--output-cvg=scores", "--output-bbox=boxes", "--threshold=0.25"], threshold=0.1)

#net = detectNet(argv=["--model=/home/capstone/Downloads/ppballs417images512resolution20batch25epoch/ssd-mobilenet.onnx", "--labels=/home/capstone/Downloads/ppballs417images512resolution20batch200epoch-20240329T061400Z-001/ppballs417images512resolution20batch200epoch/pytorch-ssd-res-512/models/ppballs350image512v2/labels.txt", "--input-blob=input_0", "--output-cvg=scores", "--output-bbox=boxes", "--threshold=0.25"], threshold=0.5)


#Initialize Movement

def forward():
    #print("Go Forward")
    GPIO.output(ena, GPIO.HIGH)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)

    GPIO.output(enb, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

def rightPivot():
    #print("Turn Right")
    GPIO.output(ena, GPIO.HIGH) #HIGH
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)

    GPIO.output(enb, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)


def backward():
    #print("Go Back")
    GPIO.output(ena, GPIO.HIGH)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(enb, GPIO.HIGH)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)

def right():
    print("Turn Right")
    GPIO.output(ena, GPIO.LOW) #HIGH
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)

    GPIO.output(enb, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

def left():
    print("Turn Left")
    GPIO.output(ena, GPIO.HIGH)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)

    GPIO.output(enb, GPIO.LOW) #HIGH
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)

def leftPivot():
    print("Turn Left")
    GPIO.output(ena, GPIO.HIGH)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)

    GPIO.output(enb, GPIO.HIGH) #HIGH
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)

def kill():
    GPIO.output(ena, GPIO.LOW)
    GPIO.output(enb, GPIO.LOW)

# UART Communication Implementation

def sendUART(ardcom):
    ardcomChar = str(ardcom)
    #print(f"Sending To Arduino: {ardcomChar}")
    arduino.write(ardcomChar.encode())  # Send the string to Arduino
    
    ## Wait for the response and print it
    #while True:
    #    if arduino.in_waiting > 0:
    #        response = arduino.readline().decode('utf-8').rstrip()
    #        print(f"Received: {response}")
    #        break
    
  
#def sendUART():
#    print("Read The RX pin from arduino")
#    # Serial Commands

#Begin Object Detection


counter = 0

def main():
	#print("Conduct movement, call sendUART when needing to send to arduino. Arduino will have recieve UART.")
	GPIO.cleanup()
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(ena, GPIO.OUT)    # PWM EN1
	GPIO.setup(in1, GPIO.OUT)    # MOTOR 1 FORWARD
	GPIO.setup(in2, GPIO.OUT)    # MOTOR 1 BACKWARD
	GPIO.setup(enb, GPIO.OUT)    # PWM EN2
	GPIO.setup(in3, GPIO.OUT)    # MOTOR 2 FORWARD
	GPIO.setup(in4, GPIO.OUT)    # MOTOR 2 BACKWARD

	camera = videoSource("csi://0")      # '/dev/video0' for V4L2
	display = videoOutput("blockTest.mp4") # 'my_video.mp4' for file

	sendUART(3)
	class_name = None

	while display.IsStreaming():
		img = None
		try:
			img = camera.Capture()
		except Exception as e:
			print("Exception: ",e)
		if img is None: # capture timeout
			continue
		

		detections = net.Detect(img)
		class_name = None
		for objects in detections:
			maxObj = None
			for i in detections:
				if (i.ClassID == 1):
					maxObj = i
					break
			if maxObj == None:
				leftPivot()
			else:
				for x in detections:
					if (x.Area > maxObj.Area) and (x.ClassID == 1):
						maxObj = x
				detection = maxObj
				#print ("Detection Array:")
				#print (detection.Area)
				numBall = detections[0].ClassID
				#print (numBall)
				class_name = net.GetClassDesc(detection.ClassID)
				center = detection.Center
				topBox = detection.Top
				bottomBox = detection.Bottom
				leftBox = detection.Left
				rightBox = detection.Right
				width = rightBox - leftBox
				#print("width: "+str(width))
				print(class_name + " X="+ str(center[0]) + " Y="+ str(center[1]))
				print (topBox)
				#print (bottomBox)
				#if ((class_name == "shoes") and (bottomBox > 580)):
					#timeToMove = width*0.002
					#print("Obstacle!")
					#leftPivot()
					#time.sleep(0.95)
					#forward()
					#time.sleep(timeToMove)
					
				if ((class_name == "orange_pp") and (topBox > 550)):
					print("Center")
					kill()
					center = detection.Center
					if center[0] > 900:


						#forward()
						right()
					elif center[0] < 420:
						#forward()
						left()
					else:
						print("Sweeping!")
						sendUART(1)
						forward()
						time.sleep(0.95)
						sendUART(3)
						class_name = None
						
					#sendUART(3)
				elif ((center[0] > 870) and (class_name == "orange_pp")):
					#sendUART(3)
					#print ("turning right")
					right()
				elif ((center[0] < 430) and (class_name == "orange_pp")):
					#sendUART(3)
					#print ("turning left")
					left()
				elif ((class_name == "orange_pp") and (topBox <=540)):
					#sendUART(3)
					#print ("Going Forward")
					forward()
				#elif detections == None:
					#print ("no detection/close")
					#left()
					#kill()
					#sendUART(3)
					#break

			
		# kill()
		if class_name != "orange_pp": #If cant find any ping pong balls
			print ("no detection/close")
			leftPivot()
			#kill()			
			

		#print(detections.ClassID)

		display.Render(img)
		display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))


if __name__ == '__main__':
    main()
