import serial
import time
from plot import GPSPlotter

arduino_port = "COM3" # Change to your port
baud_rate = 9600
time_start = time.time()
output_file = f"logs/GPS_Log_{time_start}.txt"

print(f"Connecting to {arduino_port} at {baud_rate} baud.")
ser = serial.Serial(arduino_port, baud_rate)
print("Connected to serial port.")

plotter = GPSPlotter()

try:
    print("Saving data to file...")
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode("utf-8").strip()
            plotter.plot(data)
            print(data)
            with open(output_file, "a", encoding="utf-8") as file:
                file.write(data+"\n")
except KeyboardInterrupt:
    print("\nStopping data capture.")
finally:
    ser.close()
    print("Serial port closed.")
    plotter.save(f"logs/GPS_Plot_{time_start}.png")
    plotter.show()
    # plotter.close()
