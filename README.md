# 🛡️ IoT Anti-Theft Security System

A real-time, computer-vision-powered anti-theft and physical security system. This project integrates low-level hardware (ESP32-CAM) with state-of-the-art machine learning (YOLOv8) to provide instantaneous threat detection and alerting capabilities.

## 🚀 Project Overview

Traditional security systems rely on passive recording. This system was engineered to be proactive, utilizing an ESP32-CAM module to stream live video data to a centralized processing unit where YOLOv8 object detection algorithms analyze the feed in real-time. Upon detecting unauthorized entities or suspicious activities, the system immediately triggers automated alerts.

### 👑 Leadership & Team Management

This project was developed at scale. As the **Technical Team Leader**, I architected the core system and successfully coordinated a cross-functional team of **28 developers and engineers**.

**Key Management Responsibilities:**

- Directed the hardware-software integration pipeline.
- Delegated tasks across embedded systems, machine learning, and backend sub-teams.
- Managed version control, code reviews, and final deployment standards.

## 🛠️ Technology Stack

- **Hardware:** ESP32-CAM Module, Microcontrollers
- **Embedded Software:** C++, Arduino IDE
- **Computer Vision:** YOLOv8 (Ultralytics)
- **Backend & Processing:** Python, OpenCV

## ⚙️ System Architecture

1. **Image Acquisition:** The ESP32-CAM captures high-framerate video data over a local network connection.
2. **Data Transmission:** Video frames are transmitted via HTTP/WebSocket protocols to the processing server.
3. **Real-Time Inference:** A Python backend intercepts the stream, applying a custom-configured YOLOv8 model to identify predefined physical threats or unauthorized entry.
4. **Alerting Mechanism:** Upon positive detection thresholds, the system triggers subsequent security protocols (e.g., local alarms, network notifications).

![System Architecture Diagram](link-to-your-image.png)

## 📸 Demonstration

![Demo GIF](link-to-your-demo.gif)

## 💻 Installation & Setup

### Hardware Requirements

- 1x ESP32-CAM Module
- FTDI Programmer (for initial flashing)
- Power Supply (5V/2A recommended for stability during transmission)

### Software Deployment

1. **Clone the repository:**
   ```bash
   git clone https://github.com/zoghby-ctrl/iot-anti-theft.git
   cd iot-anti-theft
   ```

2. **Flash the ESP32-CAM:**
   - Open `esp32_camera_server.ino` in the Arduino IDE.
   - Update your SSID and Password credentials.
   - Flash the code to the board and retrieve the assigned IP address from the Serial Monitor.

3. **Setup the Python Environment:**
   ```bash
   pip install -r requirements.txt
   ```

4. **Run the Inference Engine:**
   ```bash
   python main.py --stream_url http://<YOUR_ESP32_IP_ADDRESS>
   ```

## 🧠 Future Optimizations

- Migrating inference to edge devices (e.g., Raspberry Pi or Jetson Nano) to reduce network latency.
- Implementing facial recognition for whitelisting authorized personnel.
- Integrating a cloud database for logging historical detection events.

## 🤝 Contributors

- **[Your Name/Handle]** - *Project Lead & Core Architect*
- *With deep gratitude to the 28 dedicated team members who brought this system to life.*
