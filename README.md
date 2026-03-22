# Smart Road Alert System

A real-time vehicle detection and alert system for improving driver safety on narrow, curved, and inclined roads through autonomous inter-post communication and intelligent traffic prioritization.

## Overview

Smart Road Alert is an intelligent roadside alert system designed for narrow, curved, and inclined road segments where restricted sightlines create blind spots and collision hazards. The system uses computer vision to detect approaching vehicles, estimate their speed and size, and communicate this information wirelessly between two roadside posts. Each post autonomously generates real-time driver alerts via LED displays and audio warnings.

This is a **two-node, symmetric, peer-to-peer architecture** where each post operates identically and bidirectionally, allowing drivers approaching from either direction to receive advance warning about oncoming traffic, enabling them to stop, slow down, or proceed safely.

## Key Features

- **Real-time Vehicle Detection**: YOLOv8-based object detection with multi-class classification (bike, car, truck, emergency vehicles, etc.)
- **Speed Estimation**: Continuous vehicle speed tracking from depth camera measurements
- **Wireless P2P Communication**: Bidirectional 433 MHz HC-12 radio links between roadside posts
- **Intelligent Prioritization**: Size-based vehicle classification with emergency vehicle fast-track handling
- **Kinematic-Based Emergency Detection**: Distinguishes active emergency responses from normal emergency vehicle traffic using speed, acceleration, and variance metrics
- **Multi-Modal Alerts**: Synchronized visual (LED display) and audio (voice/tone) warnings
- **Solar-Powered Deployment**: Off-grid capable with battery storage and MPPT charge management

## System Architecture

The system consists of two independent roadside posts (Post A and Post B) at opposite ends of a road segment. Each post operates identically with the same hardware and firmware.

```
Post A                          HC-12 433 MHz                     Post B
├─ OAK-D Lite Camera           ═════════════════────────────     ├─ OAK-D Lite Camera
├─ Raspberry Pi 5              (Bidirectional)                    ├─ Raspberry Pi 5
├─ ESP32 + HC-12 Radio                                            ├─ ESP32 + HC-12 Radio
├─ P10 LED Panel                                                  ├─ P10 LED Panel
└─ Audio Module                                                   └─ Audio Module
```

### Data Flow Pipeline

```
OAK-D Lite Camera
    ↓
Raspberry Pi 5 (Detection, Speed Estimation, Packaging)
    ↓
ESP32 (HC-12 Bridge)
    ↓
HC-12 Wireless Transmission (433 MHz)
    ↓ (over air)
    ↓
Remote ESP32 (HC-12 Reception)
    ↓
Remote Raspberry Pi 5 (Decision Logic & Prioritization)
    ├─→ ESP32 (Display Commands)
    │   └─→ P10 LED Panel
    └─→ Audio Module
```

This pipeline operates **bidirectionally and simultaneously**, with each post acting as both transmitter and receiver.

## Hardware Components

### Bill of Materials

| Component | Quantity | Purpose |
|-----------|----------|---------|
| Raspberry Pi 5 | 2 | Main processing (host controller) |
| ESP32 DevKitC | 2 | Display driver and HC-12 bridge |
| OAK-D Lite Camera | 2 | Vehicle detection and distance measurement |
| HC-12 Transceiver | 2 | 433 MHz wireless P2P communication |
| P10 LED Panel (32x16) | 6 (2 per post) | Visual driver alerts |
| DFPlayer Mini | 2 | Audio alert playback |
| Raspberry Pi SD Card | 2 | OS and application storage |
| Solar Panel | 2 | Energy Generation |
| LiFePO4 Battery (12V 50Ah) | 2 | Energy storage |
| SN74HC245N Octal Buffer | 5 | DC level shifting |

### Key Hardware Details

**Raspberry Pi 5**: Central processing unit for each post. Runs vehicle detection inference, speed estimation, wireless communication management, and decision logic.

**ESP32**: Dedicated microcontroller acting as a bridge between the Raspberry Pi (USB) and HC-12 radio. Also drives the P10 LED display independently to prevent display latency from blocking main processing.

**OAK-D Lite Camera**: Stereo depth camera with onboard Myriad X VPU. Provides high-resolution frames for detection and real-time depth (distance) measurements for speed calculation.

**HC-12 Transceiver**: Long-range, low-power 433 MHz serial radio module. Operates in transparent mode with 9600 baud UART connection to ESP32. Default channel: 433.4 MHz, Power: 20 dBm (100 mW maximum range).

**P10 LED Panel**: 32x16 high-brightness full-color LED matrix (HUB75 interface). Displays vehicle type, estimated speed, and action signal (GO, GO SLOW, STOP).

**DFPlayer Mini**: Compact MP3 player module. Plays pre-recorded voice alerts and tones from microSD card.

## Software Components

### Codebase Structure

```
src/
├── esp32/
│   └── SmartRoadAlertClient/
│       ├── SmartRoadAlertClient.ino      # Main firmware
│       ├── PINS_CONFIG.h                 # GPIO mappings
│       ├── SERIAL_CONFIG.h/cpp           # USB host communication
│       ├── HC12_CONFIG.h/cpp             # 433 MHz radio bridge
│       └── P10_LED_CONFIG.h/cpp          # Display control
│
└── rpi/
    ├── SmartRoadAlertHost/
    │   ├── main.py                       # Host controller entry point
    │   ├── serial_config.py              # ESP32 serial management
    │   └── requirements.txt              # Python dependencies
    │
    └── references/                       # Documentation and examples
```

### Raspberry Pi 5 Host (Python)

**main.py** - Entry point for the Smart Road Alert host application.

Responsibilities:
- Initialize and manage USB serial connection to ESP32 (SerialManager)
- Initialize and manage HC-12 wireless communication
- Run YOLOv8 camera inference in a background thread
- Process inbound messages from both ESP32 and remote post via HC-12
- Apply prioritization and decision logic
- Route display and audio commands
- Send periodic heartbeats and status requests

**Key Features**:
- Fully symmetric peer-to-peer architecture (both RPis run identical code)
- Thread-safe message queues for bidirectional communication
- Automatic ESP32 port discovery and reconnection
- Real-time camera inference (YOLOv8n at ~20 FPS)
- Graceful SIGINT/SIGTERM shutdown

**serial_config.py** - SerialManager module for USB communication with ESP32.

Handles:
- Automatic ESP32 device discovery (VID:PID matching)
- Handshake protocol (HELLO/ESP32_READY)
- Non-blocking line reader thread
- Connection monitoring and automatic reconnection
- Thread-safe message queue

### ESP32 Firmware (C++)

**SmartRoadAlertClient.ino** - Main firmware for ESP32.

Responsibilities:
1. Maintain USB-serial handshake link with host RPi (host heartbeat detection)
2. Drive the P10 HUB75 LED matrix display
3. Act as a transparent bridge for HC-12 radio traffic:
   - Receive `HC12_SEND` messages from RPi, transmit payload via HC-12
   - Receive HC-12 messages, wrap and forward to RPi as `HC12_RECV`
4. Forward vehicle telemetry data to RPi

**Communication Protocol**:
```
RPi → ESP32 (USB):
  {"type":"HC12_SEND","payload":"<escaped-json>"}

ESP32 → HC-12:
  <raw-json>

HC-12 → Remote ESP32 → Remote RPi (USB):
  {"type":"HC12_RECV","payload":"<escaped-json>"}
```

**Configuration Headers**:
- `PINS_CONFIG.h`: GPIO pin assignments for display, HC-12, serial
- `SERIAL_CONFIG.h`: USB communication parameters (baud, timeouts, handshake)
- `HC12_CONFIG.h`: HC-12 module settings (AT commands, baud, power)
- `P10_LED_CONFIG.h`: LED display refresh rates and rendering functions

## Communication Protocol

### HC-12 Wireless Link (433 MHz)

- **Frequency**: 433.4 MHz (default channel)
- **Baud Rate**: 9600 bps (factory default)
- **Power**: 20 dBm (100 mW, maximum range)
- **Range**: Up to 1 km in open field conditions
- **Message Format**: JSON over transparent UART

### Serial Communication Layers

```
Host RPi ←→ [USB Serial 115200 baud] ←→ ESP32 ←→ [UART 9600 baud] ←→ HC-12 Radio
```

The ESP32 acts as a bridge, transparently forwarding HC-12 messages while also managing P10 LED display updates independently.

### Message Types

**Vehicle Detection** (Local → Remote):
```json
{
  "type": "vehicle",
  "class": "truck",
  "speed": 25.5,
  "distance": 42.3,
  "timestamp": 1234567890.5
}
```

**RPI Heartbeat** (P2P):
```json
{
  "type": "RPI_PING",
  "node": "rpi-001"
}
```

**Display Command** (RPi → ESP32):
```json
{
  "cmd": "display",
  "state": "STOP",
  "vehicle": "truck",
  "speed": 25.5
}
```

## Vehicle Detection & Speed Estimation

### Detection Methods

The system supports two detection configurations:

**YOLOv8n (Recommended)**: YOLOv8 nano model running on Raspberry Pi 5 via ultralytics library. Achieves 20+ FPS on RPi 5 with headless deployment.

**DepthAI Pipeline** (Optional): YOLO inference running on OAK-D Lite's Myriad X VPU, enabling on-camera inference and spatial coordinate extraction. Reduces RPi CPU load at the cost of less flexible model updates.

### Supported Vehicle Classes

- **Small**: Bicycle, Motorcycle, EV Small
- **Medium**: Car, Tricycle, Tuktuk, Kariton, Police Car
- **Large**: Van, Jeepney, Bus, Truck, Ambulance, Fire Truck

### Speed Estimation Algorithm

Speed is estimated by tracking depth changes across consecutive frames:

```
speed (km/h) = (Δdistance / Δtime) × 3.6
```

Where:
- Δdistance = change in depth (meters) between consecutive frames
- Δtime = time elapsed between frames (seconds)
- 3.6 = conversion factor from m/s to km/h

Speed measurements are collected over a sliding observation window (0.5-1.0 second) to compute:
- **Mean Speed**: For direct alert decisions
- **Speed Variance** (std dev): For emergency vehicle kinematic detection
- **Acceleration**: For emergency response inference

## Decision Logic & Alert System

### Vehicle Prioritization (Normal Mode)

In normal (non-emergency) operation, the system prioritizes vehicles by size class:

```
Priority 1 (STOP) : Large Vehicles    (Van, Jeepney, Bus, Truck)
Priority 2 (WARN) : Medium Vehicles   (Car, Tricycle, Police Car)
Priority 3 (INFO) : Small Vehicles    (Motorcycle, Bicycle)
```

**Rationale**: Larger vehicles occupy more road width, require longer stopping distances on inclines, and create greater obstruction on curves. Therefore, larger vehicles receive right-of-way to minimize collision risk.

### Alert States

The P10 display shows one of three driver action signals:

- **GO**: Road is clear or low-obstruction vehicle detected. Drivers may proceed normally.
- **GO SLOW**: Medium-priority vehicle detected. Drivers should proceed with caution at reduced speed.
- **STOP**: Large vehicle detected or high-obstruction condition. Drivers must wait.

### Emergency Mode Detection

Emergency vehicles (ambulance, fire truck, police car) trigger **Emergency Mode** when detected with kinematic evidence of active emergency response. Three independent conditions can activate Emergency Mode:

**1. Speed Threshold**:
```
Detected Speed >= (Average Road Speed + Threshold)
```
Thresholds (road-type specific):
- Narrow or Curved: +10 km/h above average
- Inclined: +15 km/h above average
- Combined: +15 km/h above average

**2. Aggressive Acceleration**:
```
Longitudinal Acceleration >= 1.5 m/s² sustained for ≥ 2 seconds
```

**3. Speed Variance**:
```
Standard Deviation of Speed >= Variance Threshold (3-5 km/h default)
```

Emergency Mode requires **both emergency vehicle class AND at least one kinematic condition**. This dual-evidence requirement prevents false positives from static emergency vehicles or brief non-emergency speed bursts.

**Algorithm** (simplified):
```
Emergency Mode = (Emergency Vehicle Class Detected)
                 AND
                 [
                   (Speed >= Avg + Threshold)
                   OR (Acceleration >= 1.5 m/s²)
                   OR (Variance >= Threshold)
                 ]
```

### Emergency Mode Behavior

When Emergency Mode is activated:
- Emergency vehicle receives highest priority, overriding normal size-based prioritization
- Ambulances supersede all other vehicles including trucks
- Fire trucks prioritized based on speed/acceleration conditions (not variance alone)
- Police cars follow standard Emergency Mode logic
- Remote post immediately receives STOP signal
- LED display shows prominent emergency warning
- Audio module plays emergency alert tone

## Project Structure

```
smart-road-alert/
├── README.md                          # This file
├── LICENSE                            # MIT License
├── src/
│   ├── esp32/SmartRoadAlertClient/   # ESP32 firmware
│   │   ├── *.ino, *.h, *.cpp
│   │   └── PINS_CONFIG.h             # GPIO reference
│   │
│   ├── rpi/SmartRoadAlertHost/       # Raspberry Pi host application
│   │   ├── main.py                   # Entry point
│   │   ├── serial_config.py          # ESP32 manager
│   │   └── requirements.txt
│   │
│   └── sample/                        # Reference code & examples
│       ├── P10Led/                    # LED panel examples
│       └── yolo/                      # YOLO inference examples
│
└── wiring/
    └── SmartRoadAlertWiringDiagram.fzz  # Fritzing circuit diagram
```

## Dependencies

### Raspberry Pi 5 Host

**Python 3.8+** with the following packages:

```
pyserial>=3.5                          # USB serial communication
RPi.GPIO>=0.7.0                        # HC-12 SET pin control (GPIO17)
ultralytics>=8.0.0                     # YOLOv8n model and inference
opencv-python-headless>=4.8.0          # Camera capture (headless, no GUI)
```

Install via:
```bash
cd src/rpi/SmartRoadAlertHost
pip install -r requirements.txt
```

### ESP32 Firmware

Requires **Arduino IDE** or **PlatformIO** with:
- ESP32 board support package
- Arduino core for ESP32
- Standard libraries (Arduino.h, SoftwareSerial, etc.)

### Hardware Requirements

- Raspberry Pi 5 (2x for full system)
- ESP32 DevKitC (2x)
- OAK-D Lite camera (1x minimum, ideally 2x for testing)
- Linux OS (Raspberry Pi OS or similar)
- USB-A to Micro-B cable for ESP32 serial connection

## Configuration

### ESP32 Pin Configuration

Edit [src/esp32/SmartRoadAlertClient/PINS_CONFIG.h](src/esp32/SmartRoadAlertClient/PINS_CONFIG.h):

```cpp
#define PIN_HC12_RX   16   // HC-12 receives from this GPIO
#define PIN_HC12_TX   17   // HC-12 transmits to this GPIO
#define PIN_HC12_SET  22   // HC-12 mode select (LOW=AT, HIGH=transparent)
#define PIN_P10_A     12   // P10 address lines
#define PIN_P10_B     13
// ... (see full file for complete mapping)
```

### HC-12 Module Settings

Default configuration (factory settings):
- Mode: FU3 (transparent operation)
- Baud: 9600 bps
- Channel: CH001 (433.4 MHz)
- Power: Level 8 (20 dBm)

To configure via AT commands, set HC-12 SET pin LOW. [HC12_CONFIG.h](src/esp32/SmartRoadAlertClient/HC12_CONFIG.h) handles this automatically on startup.

### Raspberry Pi Host Configuration

Edit [src/rpi/SmartRoadAlertHost/main.py](src/rpi/SmartRoadAlertHost/main.py):

```python
ESP32_ENABLED: bool = True                    # Enable/disable ESP32
CAMERA_INFERENCE_ENABLED: bool = True         # Enable/disable camera
PING_INTERVAL_S: float = 5.0                  # Heartbeat interval
HC12_PING_INTERVAL_S: float = 5.0             # HC-12 heartbeat
NODE_ID: str = socket.gethostname()           # Node identifier
```

## Installation & Setup

### 1. Flash ESP32 Firmware

```bash
# Clone or download the repository
cd src/esp32/SmartRoadAlertClient

# Option A: Arduino IDE
# 1. Open SmartRoadAlertClient.ino
# 2. Select Board: ESP32 DevKitC
# 3. Select Port: /dev/ttyUSB0 or COM port
# 4. Click Upload

# Option B: PlatformIO (if using VSCode)
# 1. Install PlatformIO extension
# 2. Run: pio run -t upload
```

Verify upload in serial monitor (9600 baud):
```
[SmartRoadAlertClient] Ready
```

### 2. Install Raspberry Pi OS on RPi 5

- Download Raspberry Pi Imager
- Flash Raspberry Pi OS (64-bit) to SD card
- Enable SSH and I2C during imaging (optional but recommended)
- Boot and configure

### 3. Install Python Dependencies

```bash
cd src/rpi/SmartRoadAlertHost

# Create virtual environment (recommended)
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install --upgrade pip
pip install -r requirements.txt

# Note: RPi.GPIO requires hardware GPIO access; test with:
python3 -c "import RPi.GPIO; print('GPIO OK')"
```

### 4. Test USB Connection to ESP32

```bash
cd src/rpi/SmartRoadAlertHost

# Quick connection test
python3 -c "from serial_config import SerialManager; m = SerialManager(); m.start(); print('Connected:', m.is_connected()); m.stop()"
```

Expected output:
```
15:32:01 [INFO] serial_config: Attempting to connect to ESP32...
15:32:02 [INFO] serial_config: Connection established; handshake successful.
Connected: True
```

### 5. Run Host Application

```bash
cd src/rpi/SmartRoadAlertHost
python3 main.py
```

Expected console output:
```
2026-03-22 15:32:15 [INFO] SmartRoadAlert: Smart Road Alert Host starting.
2026-03-22 15:32:16 [INFO] SmartRoadAlert: Camera inference thread started.
2026-03-22 15:32:16 [INFO] SmartRoadAlert: Main loop running. Press Ctrl+C to stop.
```

## Usage & Operation

### System Operation

Once both posts are running, they automatically:

1. **Detect Vehicles**: YOLOv8 runs at ~20 FPS, classifying all visible vehicles
2. **Estimate Speed**: Continuous depth tracking produces real-time speed estimates
3. **Exchange Data**: Each post wirelessly transmits its local detections to the remote post via HC-12
4. **Apply Logic**: Receive local and remote data, prioritize by size/emergency status
5. **Display Alerts**: Update LED panel and play audio warnings based on decision logic

### Example Scenario

**Scenario**: A motorcycle (low priority) approaches Post A from the left at 12 km/h; no vehicles at Post B.

**Post A Actions**:
1. Detects motorcycle via OAK-D Lite
2. Estimates speed: 12 km/h
3. Transmits via HC-12: `{"type":"vehicle","class":"motorcycle","speed":12,"distance":25}`
4. Applies logic: low-priority vehicle, safe passage
5. Sends to ESP32: `{"cmd":"display","state":"GO","vehicle":"motorcycle"}`
6. P10 LED shows: motorcycle icon, "12 km/h", "GO" signal
7. Audio module: silent (GO state)

**Post B Actions**:
1. Receives motorcycle data from remote Post A via HC-12
2. No local detections
3. Applies logic: single low-priority vehicle, safe for drivers on Post B side
4. Sends to ESP32: `{"cmd":"display","state":"GO SLOW"}`
5. P10 LED shows: motorcycle icon, "GO SLOW" signal
6. Audio module: "Vehicle approaching, proceed with caution"

### Emergency Vehicle Example

**Scenario**: An ambulance detected at Post A traveling at 40 km/h (average road speed: 20 km/h).

**Detection Phase**:
1. YOLOv8 detects class: "ambulance"
2. Speed estimate: 40 km/h
3. Acceleration: 1.8 m/s² (sustained)
4. Emergency Mode Trigger: YES (speed >= 20 + 15, acceleration >= 1.5 m/s²)

**Response**:
- Post A LED: Shows emergency warning + "STOP" signal
- Post A Audio: "EMERGENCY VEHICLE - PLEASE STOP"
- HC-12 transmits: `{"type":"vehicle","class":"ambulance","emergency":true,"speed":40}`
- Post B LED: Shows emergency warning + "STOP" signal (highest priority override)
- Post B Audio: "Emergency vehicle approaching - stop immediately"

### Monitoring & Debugging

Check system status in real-time:

```bash
# Monitor console output (already visible during main.py execution)
# Press Ctrl+C to gracefully stop

# Check HC-12 communication:
# Monitor serial output on both RPis to verify wireless packet exchange

# Test camera:
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('Camera:', cap.isOpened())"

# Check system logs:
journalctl -u smartroadaler -f  # If installed as systemd service
```

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| ESP32 not detected | USB not connected or wrong SO baud rate | Check /dev/ttyUSB* ports, reinstall drivers |
| Handshake timeout | Serial connection unstable | Try higher baud rate, shorter USB cable |
| No HC-12 communication | Radio not powered or wrong channel | Verify power supply, check AT command config |
| YOLOv8 not starting | ultralytics package missing | `pip install ultralytics` |
| Camera not found | /dev/video0 unavailable | Check USB camera connection, list with `ls /dev/video*` |
| P10 display blank | GPIO pin mismatch or power issue | Verify PINS_CONFIG.h, check 5V supply to LED panel |

## Performance Metrics (Reference)

- **Detection FPS**: 20+ FPS (YOLOv8n on RPi 5)
- **Speed Estimation Latency**: <100 ms per frame
- **HC-12 Wireless Range**: Up to 1 km (open field)
- **HC-12 Message Latency**: 10-50 ms per message
- **LED Display Refresh**: 60 Hz (16.7 ms per frame)
- **Decision Logic Latency**: <50 ms (prioritization + routing)

**Total System Latency**: ~200-300 ms from detection at Post A to alert display at Post B (under ideal conditions).

## Future Improvements

- **DepthAI Pipeline**: Offload detection to OAK-D Lite VPU for lower RPi CPU load
- **Machine Learning**: Train custom YOLO model on local road vehicle dataset for improved accuracy
- **Sensor Fusion**: Combine camera with radar or LiDAR for robust detection in adverse weather
- **Advanced Analytics**: Track vehicle history, detect unusual patterns, generate statistical reports
- **Web Dashboard**: Remote monitoring of both posts via HTTP/MQTT
- **Mobile App**: Driver notifications via Bluetooth or cellular link
- **Enhanced Audio**: Context-aware alerts ("Truck from left approaching, slow down")

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) file for details.

## Authors

Developed by Sajed Lopez Mendoza (2026)

---
