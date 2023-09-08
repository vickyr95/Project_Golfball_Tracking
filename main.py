import serial
import time
import helper.Stereo_Ball_XYZ_IMU as ball
from datetime import datetime

# Initialize serial connection
ser = serial.Serial('/dev/ttyUSB0', 115200)  # Replace 'COM3' with the port your ESP32 is connected if your system is windows. /dev/ttyUSB0 or /dev/ttyUSB1 if linux
ser.flushInput()

ball_detector = ball.BallDetector()
i_time=0
try:
    print("en")
    while True:
        # Read one byte from the serial buffer
        if ser.in_waiting > 0:
            byte_data = ser.read(1)
            
            # Convert byte to integer (ASCII to integer)
            int_data = int.from_bytes(byte_data, "little")
            
            # Convert integer to boolean (0 or 1)
            bool_data = bool(int_data)
            
            print(f"Received: {int_data} (bool: {bool_data})")
            if bool_data == 1:
                i_time=datetime.now().timestamp()

            #print(i_time)
        ball_detector.ball_detection(i_time)
            
        # Add a delay to reduce CPU usage
        time.sleep(0.01)
        
except KeyboardInterrupt:
    print("Exiting...")
    ser.close()
