# Crash Node - Automated Accident Detection & Alert System

![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white)


An Arduino-based system that automatically detects a vehicle accident and sends an emergency alert with GPS coordinates to a pre-defined number.

---

## Key Features

- **Real-time Impact Detection**: Uses an MPU-6050 accelerometer and gyroscope to detect sudden impacts and rollovers.
- **Precise GPS Tracking**: Acquires exact geographic coordinates of the incident using a NEO-6M GPS module.
- **Automated Alerts**: Instantly sends an SMS with a Google Maps link and places a voice call to an emergency contact via a SIM800L GSM module.


---

## Hardware Components

| Component              | Purpose                     |
| :--------------------- | :-------------------------- |
| Arduino Uno            | Main Microcontroller        |
| MPU-6050 Module        | Impact & Gyroscope Sensing  |
| GPS NEO-6M Module      | Geolocation Tracking        |
| SIM800L GSM Module     | SMS & Call Communication    |
| LM2596 Module          | Voltage stabilizer for GSM  |
| External Power Supply  | Stable Power for GSM Module |

---

## System Flowchart

This diagram illustrates the logical flow of the system from initialization and monitoring to alert dispatch.

<img src="Public\FlowDiagram.png" alt="System Flowchart" width="500" height="800">

---

## Circuit Diagram

This diagram shows the complete wiring for all hardware components.

<img src="Public\circuit_diagram.png" alt="System Flowchart" width="800">

---

## How It Works

The system operates on a simple state machine:

1.  **Initialization**: On startup, the Arduino initializes all sensors and modules (MPU-6050, GPS, GSM).
2.  **Monitoring State**: The system enters a continuous loop, reading data from the MPU-6050 to check for forces that exceed a pre-defined threshold.
3.  **Alert State**: If an impact is detected, the system gets a GPS lock and then uses the GSM module to send an SMS and make a call.
4.  **Failsafe State**: After the alert is sent, the program enters an infinite loop (`while(true);`) to halt all operations until it is manually reset.

---

## Software & Libraries

- [Arduino IDE](https://www.arduino.cc/en/software) 
- [TinyGPSPlus](http://arduiniana.org/libraries/tinygpsplus/) by Mikal Hart
- [Adafruit MPU6050](https://github.com/adafruit/Adafruit_MPU6050) by Adafruit
- [Adafruit Unified Sensor](https://github.com/adafruit/Adafruit_Sensor) by Adafruit
- `<Wire.h>` (Standard Arduino Library)

---

## Setup & Installation

1.  **Connect Hardware**: Wire all the components according to the circuit diagram above.
2.  **Install Libraries**: In the Arduino IDE, go to `Sketch > Include Library > Manage Libraries...` and install the required libraries listed above.
3.  **Configure Code**: Open the `.ino` sketch file and change the `EMERGENCY_NUMBER` constant to your desired phone number.
4.  **Upload**: Select your board (Arduino Uno) and COM port, then upload the sketch.

---

## License

This project is licensed under the MIT License.
