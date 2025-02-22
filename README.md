# GNSS Logger

## Overview

This Python script is designed for a GNSS (Global Navigation Satellite System) logging system. It reads data from GPS modules, records video from a camera module, and allows switching between GPS modules. It also provides a graphical user interface (GUI) for user interaction.

## Features

-   Reads data from GPS modules via serial or subprocess.
-   Records video from a Raspberry Pi Camera Module or a USB camera.
-   Switches between GPS modules using GPIO.
-   Captures images on button press or GUI command.
-   Provides a GUI for controlling recording, capturing images, and switching GPS modules.
-   Logs raw GPS data and processed location information to separate files.
-   Automatic time synchronization using GPS data.
-   Automatic video recording when GPS data is valid.

## Dependencies

-   Python 3.9 or later
-   OpenCV (`opencv-python`)
-   NumPy (`numpy`)
-   RPi.GPIO (`RPi.GPIO`)
-   Picamera2 (`picamera2`)
-   PySerial (`pyserial`)
-   Tkinter (`tkinter`)

## Setup

1.  **Install Dependencies:**

    ```bash
    pip install opencv-python numpy picamera2 pyserial tkinter
    ```

2.  **Hardware Setup:**

    -   Connect the GPS modules to the Raspberry Pi's serial port (`/dev/ttyAMA0`).
    -   Connect a Raspberry Pi Camera Module or a USB camera.
    -   Connect a button to GPIO pin 16.
    -   Connect the GPS module selection pins to GPIO pins 6 and 5.

3.  **Run the Script:**

    ```bash
    python3 main.py
    ```

    The script will start reading GPS data and automatically begin video recording when valid GPS data is received.

## Usage

### GUI Controls

-   **GPS Label:** Displays the currently active GPS module (Internal or External).
-   **Switch GPS:** Toggles between the internal and external GPS modules.
-   **Start Recording:** Manually starts video recording if it is not already automatically recording.
-   **Capture an image:** Captures and saves an image to the `logs/<timestamp>/captured_img` directory.
-   **Stop logging:** Stops all logging and video recording, and exits the program.

### Hardware Button

-   Press the connected button (GPIO 16) to capture an image.

### Logging

The logging system creates a folder structure like this:
```
logs/
|- <log time>/
|  |- captured_img/        # Contains captured images named with their timestamps.
|  |  |- <captured_time>
|  |  |- <captured_time>
|  |  |- ...
|  |- captured_time.txt    # Lists the timestamps of captured images.
|  |- logger.mp4           # The recorded video file.
|  |- video_timestamp.csv  # Timestamps of video frames.
|  |- raw_data.txt         # Raw GPS data.
|  |- location.txt         # Processed location data (latitude, longitude, altitude, easting, northing).
|  |- err.txt              # Error logs (only created if `SAVE_ERROR` is set to `True`).
|- <log time>
|- ...
```
-   Raw GPS data is logged to `logs/<timestamp>/raw_data.txt`.
-   Processed location data (latitude, longitude, altitude, easting, northing) is logged to `logs/<timestamp>/location.txt`.
-   Captured images are saved to `logs/<timestamp>/captured_img/`.
-   Video recordings are saved to `logs/<timestamp>/logger.mp4`.
-   Video timestamps are saved to `logs/<timestamp>/video_timestamp.csv`.
-   Captured image timestamps are saved to `logs/<timestamp>/captured_time.txt`.
-   Error logs (if `SAVE_ERROR` is set to `True`) are saved to `logs/<timestamp>/err.txt`.

## Code Structure

-   `main.py`: The main script that initializes and runs the GNSS logger.
-   `converter(Latitude, Longitude)`: Converts latitude and longitude to easting and northing.
-   `cameraRecord()`: Handles video recording from the camera.
-   `serialRead()`: Reads and processes GPS data.
-   `GNSSLoggerUI`: Defines the GUI class.
