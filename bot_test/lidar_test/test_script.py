import sys
import time
from rplidar import RPLidar
import numpy as np

# Set the correct serial port (adjust as necessary)
PORT_NAME = "/dev/ttyUSB0"
lidar = RPLidar(PORT_NAME)

# Distance threshold (in mm) for detecting close objects
CLOSE_OBJECT_THRESHOLD = 1000  # 1 meter = 1000 mm

def print_data():
    """Function to print data of objects within the threshold."""
    print("🚀 Scanning for close objects...")

    try:
        while True:
            for scan in lidar.iter_scans():
                # Arrays to hold angles and distances
                angles = []
                distances = []

                # Collect data
                for (_, angle, distance) in scan:
                    angles.append(angle)
                    distances.append(distance)

                # Convert to numpy arrays for efficient processing
                angles = np.array(angles)
                distances = np.array(distances)

                # Filter close objects within the threshold
                close_objects = distances < CLOSE_OBJECT_THRESHOLD
                close_angles = angles[close_objects]
                close_distances = distances[close_objects]

                # Print results if there are any close objects
                if len(close_distances) > 0:
                    for angle, distance in zip(close_angles, close_distances):
                        print(f"🔍 Close Object: Angle = {angle:.2f}°, Distance = {distance:.2f} mm")
                        if 0 < angle:
                            if angle < 150:
                                if distance < 500:
                                    print("OBJECT DETECTED")
                else:
                    print("No close objects detected.")

                

    except KeyboardInterrupt:
        print("\n🛑 Stopping Lidar and scanning...")
        lidar.stop_motor()
        lidar.disconnect()

def start_motor():
    """Function to start the Lidar motor."""
    print("Starting LIDAR motor...")
    lidar.start_motor()

def stop_motor():
    """Function to stop the Lidar motor."""
    print("Stopping LIDAR motor...")
    lidar.stop_motor()
    time.sleep(10000)

def main():
    """Main function to handle numeric arguments and perform actions."""
    
    if len(sys.argv) < 2:
        print("Usage: python3 test_script.py <number>")
        print("1 - Start motor")
        print("2 - Stop motor")
        print("3 - Print data")
        sys.exit(1)

    # Get the argument
    action = int(sys.argv[1])

    if action == 1:
        start_motor()
    elif action == 2:
        stop_motor()
    elif action == 3:
        print_data()
    else:
        print("Invalid argument. Please use 1, 2, or 3.")
        sys.exit(1)

if __name__ == "__main__":
    main()

