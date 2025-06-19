# Vital parameters sensing system

## Overview

This project was developed as part of an engineering thesis and presents a **multi-sensor data acquisition and transmission system** based on the STM32F4 microcontroller platform. The collected data is transmitted via UART to an ESP32 module, which provides wireless connectivity (Wi-Fi) for remote data access and storage.

The system features real-time data acquisition, threshold-based alerts, checksum-verified communication, and timestamped event logging - making it suitable for remote patient monitoring applications.

### Features
- Collects data from the following sensors:
  - **MLX90614**: Infrared thermometer
  - **DFRobot MAX30100**: Blood oxygen (SpO2) and heart rate
  - **GSR Sensor**: Measures skin conductance (Galvanic Skin Response)
  - **ECG 3 Click**: Captures raw ECG waveforms (electrocardiogram signals) for heart activity analysis
- Computes basic checksums for data integrity
- Sends sensor data to ESP32 over UART
- Receives RTC timestamp from ESP32 and uses it to timestamp measurements
- Switches sensors via user button and logs selected mode over UART
- Implements simple retry logic and ACK/NACK protocol

---

## Hardware Requirements

- **STM32F4 Discovery Board** (STM32F429I-DISC1)
- **ESP32** (used for communication with an external server or cloud via Wi-Fi)
- **MLX90614 IR Temperature Sensor**
- **DFRobot MAX30100 Pulse Oximeter Sensor**
- **GSR Sensor**
- **ECG 3 Click**
- **I2C** and **UART** wiring for sensors and ESP32
- **Button** to switch between sensor modes

<!-- --- -->

<!-- ## Software Components

- **STM32 HAL Drivers**
- **FreeRTOS**
- **Custom sensor drivers** for MLX90614 and MAX30100
- **UART communication protocol** with retry, ACK/NACK, and CRC validation
- **Checksum function** for verifying payload integrity
- **RTOS Tasks**:
  - `defaultTask`: USB Host (optional)
  - `myTask`: Main telemetry loop for sensor selection and transmission -->

---

## Sensor Switching

Pressing the **user button** (`PA0`) cycles through the three sensor modes:

| Mode | Sensor          | Description                    |
|------|------------------|--------------------------------|
| 0    | MLX90614         | IR thermometer                 |
| 1    | MAX30100         | SpO2 and heart rate            |
| 2    | GSR              | Skin conductance (stress level)|
| 3    | ECG              | ECG waveforms                  |

The currently selected sensor is displayed via UART log and toggled LED.

---

## UART Protocol

### STM32 ➝ ESP32

JSON-formatted payload with checksum:

```json
{"timestamp":17283719,"ambient":23.50,"object":35.74,"crc":123}
```

### ESP32 -> STM32

On receiving a valid payload, ESP32 should reply with either:

- `ACK` – if received and parsed correctly
- `NACK` – if checksum is invalid or error occurs

---

## RTC Synchronization

- STM32 sends `GET_RTC` to ESP32 during startup
- ESP32 replies with a JSON string like:
  ```json
  {"rtc":17283719}
  ```
- STM32 parses and uses this timestamp as a base time for sensor data

---

## ESP32 Firmware

The ESP32 acts as a communication bridge between the STM32 and the Internet. It connects to a Wi-Fi network and synchronizes time via NTP, providing real-time clock (RTC) information to the STM32 at startup or upon request. It also validates incoming data payloads from the STM32 using a checksum and responds with `ACK` or `NACK` accordingly.

### Features
- Connects to Wi-Fi (SSID and password configured in code)
- Synchronizes with NTP server (`pool.ntp.org`) to get accurate UTC time
- Sends initial RTC timestamp to STM32 on boot
- Responds to `GET_RTC` command from STM32 with current epoch time in JSON format
- Parses incoming JSON payloads from STM32 and verifies checksum
- Responds with:
  - `ACK` if checksum is valid
  - `NACK` if checksum is incorrect or malformed
- Uses UART2 (`RXD2 = GPIO 16`) to communicate with STM32
- Basic retry logic for outgoing messages (up to 3 retries)

### Example Payload Sent to STM32
```json
{"rtc":17283719,"crc":201}
 ```
 ---


<!-- // ## Build & Flash Instructions

// 1. Open the project in STM32CubeIDE or any compatible development environment.
// 2. Connect the STM32 board via USB.
// 3. Build and flash the firmware.
// 4. Open a serial terminal on `USART1` (115200 baud) to view logs.
// 5. Ensure ESP32 is ready on `UART5` (9600 baud) to respond to RTC and sensor messages.

// --- -->

