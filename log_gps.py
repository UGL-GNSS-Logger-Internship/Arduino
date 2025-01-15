import serial
import time

arduino_port = "COM3"
baud_rate = 9600
output_file = f"GPS_Log_{time.strftime('%d%m%Y_%H%M%S')}.txt"

print(f"Connecting to {arduino_port} at {baud_rate} baud.")
ser = serial.Serial(arduino_port, baud_rate)
print("Connected to serial port.")

try:
    print("Saving data to file...")
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode("utf-8").strip()
            print(data)
            with open(output_file, "a", encoding="utf-8") as file:
                file.write(data+"\n")
except KeyboardInterrupt:
    print("\nStopping data capture.")
finally:
    ser.close()
    print("Serial port closed.")
