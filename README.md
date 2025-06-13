# Autonomous Scanning Robot with Obstacle Avoidance and Survivor Detection

This project implements an autonomous mobile robot using the iRobot Create 2 platform integrated with the Tiva C Series TM4C123GXL microcontroller. The robot is capable of navigating its environment, avoiding obstacles, scanning for objects, and classifying potential survivors (humans or rescue team members) based on distance and size metrics using both IR and ultrasonic sensors.

## 🚀 Features

- **Real-Time Movement Control**: Supports forward, backward, turning via UART commands.
- **Cliff and Bump Detection**: Avoids falling off edges and responds to physical obstacles.
- **IR & Ultrasonic Scanning**: Performs angular scanning from 0° to 180° to detect and classify objects.
- **Object Classification**: Identifies detected objects as:
  - `'H'`: Potential Human Survivor
  - `'T'`: Team Rescuer
  - `'D'`: Debris/Unknown Object
- **Autonomous Decision-Making**: Responds to detected conditions with preprogrammed behaviors.
- **UART Interface**: Communicates with PC via UART to log data and receive control commands.

## 🧰 Technologies Used

- **Embedded C**
- **Tiva C Series TM4C123GXL**
- **iRobot Create 2 API**
- **ADC for IR Sensor**
- **Ultrasonic (Ping) Sensor**
- **Custom UART Communication**
- **Real-time Event Handling and Servo Control**

## 🗂️ Project Structure

├── main.c # Main controller logic
│ ├── servo.c # Servo control logic
│ ├── ping.c # Ultrasonic distance measurement
│ ├── adc.c # IR sensor data reading
│ ├── uart_extra_help.c # UART utilities
│ └── ...
├── include/
│ ├── servo.h
│ ├── ping.h
│ ├── adc.h
│ ├── uart_extra_help.h


## 🛠️ Build & Run Instructions

> ✅ **Hardware Required:**
> - TI Tiva C LaunchPad (TM4C123GXL)
> - iRobot Create 2
> - Sharp IR Sensor
> - Ultrasonic Ping Sensor
> - Servo Motor
> - UART-to-USB cable

### 1. Prerequisites

- **Compiler:** `arm-none-eabi-gcc` (Use Code Composer Studio or Keil uVision)
- **Flashing Tool:** TI LM Flash Programmer / Energia / OpenOCD
- **Terminal (optional):** PuTTY / RealTerm for UART logs

### 2. Build

```bash
cd robot-survivor-scan
make
4. UART Commands
'w': Move forward

's': Move backward

'a': Turn left

'd': Turn right

'm': Perform environmental scan

🧪 Object Classification Logic
Classification	Width (cm)	Meaning
H	6 – 11 cm	Survivor (Human)
T	≤ 14 cm	Team Responder
D	> 14 cm	Debris / Unknown

📄 Future Improvements
Implement Bluetooth or Wi-Fi control via mobile app

Add onboard object memory mapping (SLAM)

Deploy deep learning-based object classification (e.g., with a Raspberry Pi)

📌 Authors
Rafat Momin 

Iowa State University – Computer Engineering
