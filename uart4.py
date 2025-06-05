import serial
import time

arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

time.sleep(2)

def send_and_receive(test_str):
    print(f"Sending: {test_str}")
    arduino.write(test_str.encode())  # Send the string to Arduino
    
    # Wait for the response and print it
    while True:
        if arduino.in_waiting > 0:
            response = arduino.readline().decode('utf-8').rstrip()
            print(f"Received: {response}")
            break

try:
    while True:
        # Example test string to send to the Arduino
        test_str = "Hello Arduino"
        input_str = input("input string")

        send_and_receive(input_str)

        # Wait a bit before sending the next string
        time.sleep(1)

except KeyboardInterrupt:
    print("Program exited by user")
except Exception as e:
	print("Exception error: ", e)
finally:
    arduino.close()  # Ensure the serial connection is closed on exit

