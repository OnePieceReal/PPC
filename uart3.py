import serial
import time

print("running uart3.py")

# Setup serial connection (adjust '/dev/ttyTHS1' to the correct port)
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # 

while True:
	print("entered while loop")
	try:
		print("entered try")
		data = arduino.readline()
		print(data)
		print("Data ^")
		if data:
			print(data.decode())
			print("test")	
	
	except serial.SerialException as e:
		print("Serial communication error: ",e)	
		arduino.close()
		break
	except Exception as e:
		print("arduino.close()")
		print("Error code: ",e)
		arduino.close()
		break
