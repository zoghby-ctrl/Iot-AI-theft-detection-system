# Smart Shelf: IoT Anti-Theft System
### An AI-Powered Package Security Solution

## Description
An intelligent, battery-optimized security system designed to detect and prevent package theft. It utilizes a hybrid IoT architecture where an ESP32-CAM handles sensor fusion (Vibration + Ultrasonic + PIR) to detect physical tampering, while a Python Flask Server uses YOLOv8 AI to verify human presence in captured images, eliminating false alarms.


Features
Dual-Sensor Theft Detection: Combines a Vibration Sensor (to detect force) with an Ultrasonic Sensor (to verify item removal) to prevent false positives.

AI Computer Vision: Automatically uploads captured photos to a local server where a YOLOv8 neural network scans for human presence.

Deep Sleep Power Management: The system utilizes the ESP32's Deep Sleep mode and External Wake-up (ext1) pins to maximize battery life, only waking up when sensors are triggered.

Real-Time Alerts: Triggers a local physical buzzer for immediate deterrence and logs photographic evidence to the server instantly.

Event-Driven Architecture: Code execution is optimized to run only during active threat detection, keeping the processor dormant otherwise.

Technology Stack
Hardware: ESP32-CAM (AI-Thinker), SW-18010P Vibration Sensor, HC-SR501 PIR Sensor, HC-SR04 Ultrasonic Sensor.

Microcontroller Firmware: C++ (Arduino Framework).

Backend Server: Python 3.x, Flask.

Artificial Intelligence: Ultralytics YOLOv8 (Object Detection).

Communication Protocol: HTTP POST over WiFi (802.11 b/g/n).

How to Run
Part 1: Start the Server (The Brain)
Ensure Python is installed on your machine.

Install required libraries: pip install flask ultralytics opencv-python.

Run the server script:

Bash

python SmartShelf.py
Ensure the theft_evidence folder exists in the project directory.

Part 2: Deploy the Hardware (The Sentry)
Open the firmware code in Arduino IDE.

Update the serverName variable with your computer's local IP address (e.g., 10.18.xxx.xxx).

Upload the code to the ESP32-CAM (Remember to ground GPIO 0 during upload).

Disconnect the USB and power the board via the 5V/GND pins.

Arming: The system will blink/beep to indicate startup, then enter Deep Sleep. It is now ready to detect theft.
