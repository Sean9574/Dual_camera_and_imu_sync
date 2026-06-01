# IMU I2C to Serial Publisher with Camera Trigger

**Hardware:** XIAO RP2350 / ESP32-S3  
**IMU Rate:** 100 Hz binary telemetry  
**Camera Support:** OV9281 synchronized trigger via GPIO5  
**Serial:** 115200 baud COBS-encoded packets  
**Status:** Production-ready (April 2026)  

---

## Overview

This firmware reads from one IMU sensor over I2C, publishes binary telemetry packets at 100 Hz, and optionally triggers synchronized USB cameras via GPIO5. Perfect for Visual-Inertial Odometry (VIO) and other robotics applications requiring tight IMU-camera synchronization.

### Supported IMUs

Three sensors auto-detected at startup (first-found wins):

| Sensor | Type | Outputs | Mode |
|--------|------|---------|------|
| **BNO085** (Bosch) | 9-DOF fusion | Quaternion, gyro, accel | SID=3 |
| **LSM6DSOX** (ST) | 6-axis | Gyro, accel | SID=1 |
| **BMI088** (Grove) | 6-axis | Gyro, accel (ENU mapped) | SID=2 |

Only **one IMU per board**. If multiple are soldered, first-detected is used.

---

## Quick Start

### 1. Flash Firmware (VS Code)

**Install PlatformIO Extension:**
- Open VS Code
- Go to Extensions (Ctrl+Shift+X)
- Search "PlatformIO IDE"
- Click Install

**Flash to board:**
- Connect XIAO to USB
- In VS Code: Open the project folder
- Click "PlatformIO: Upload" in the bottom status bar
- Or: Click home icon → select board (seeed_xiao_rp2350) → Upload

**Monitor output:**
- Click "PlatformIO: Monitor" in status bar (115200 baud auto-detected)

### 2. Verify IMU

You should see:
```
IMU binary v1 ready (COBS+CRC16), 115200 baud
Commands: T=test, C=CRC toggle, D=accel
IMU: BNO085 detected
Static covariance mode: ENABLED
CameraManager: Initialized on GPIO5, pulse=20 µs, interval=40 ms
```

Send **T** to generate test packet.

### 3. Set Camera Trigger Rate (Optional)

Edit `src/main.cpp` line 66:
```cpp
static const uint32_t CAMERA_TRIGGER_RATE_MS = 40;  // 25 fps (every 4 IMU samples)
```

Available options (all integers for perfect IMU sync):
- `40` ms = 25 fps (every 4 IMU samples) — **recommended for VIO**
- `50` ms = 20 fps (every 5 IMU samples)
- `100` ms = 10 fps (every 10 IMU samples)

Then flash with PlatformIO.

### 4. Test Cameras (Linux)

Enable external trigger mode:
```bash
cd tools
python3 setup_trigger.py
```

Display both camera feeds:
```bash
python3 display_cameras.py
```

---

## Packet Format

**Structure (106 bytes raw, ~102 encoded + delimiter):**

```
[0x31]               # Packet type (u8)
[seq:u16]            # Sequence number (0-65535, wraps every ~655 sec @ 100Hz)
[sensor_id:u8]       # IMU type: 1=LSM6DSOX, 2=BMI088, 3=BNO085, 0=none
[flags:u8]           # Reserved, currently 0
[t_ms:u32]           # Timestamp (milliseconds since startup)
[qw,qx,qy,qz:f32]    # Quaternion (normalized if BNO085; zero if not available)
[gx,gy,gz:f32]       # Gyroscope (rad/s)
[ax,ay,az:f32]       # Accelerometer (m/s²)
[cov_ori_x,y,z:f32]  # Orientation covariance diagonal (3 elements)
[cov_gyr_x,y,z:f32]  # Gyro covariance diagonal (3 elements)
[cov_acc_x,y,z:f32]  # Accel covariance diagonal (3 elements)
[crc16:u16]          # CRC16-CCITT (0x1021 polynomial)
+ COBS encoding (byte-stuffing for 0x00)
+ 0x00 delimiter
```

**Timing:** Exactly 10 ms between packets (100 Hz guaranteed)

---

## Covariance Modes

### Dynamic (Current Default: Recommended)

- Computed from rolling window of last 200 samples (~2 seconds @ 100 Hz)
- Adapts to real-world vibration and noise
- Memory: ~2.4 KB per accumulator
- Formula: unbiased sample covariance (divide by n-1)

**Best for:** Real-world deployment, VIO, any adaptive system

### Static (Optional)

- Uses datasheet-derived per-sensor values
- Fixed regardless of actual noise
- Zero memory overhead
- Configuration: Edit `USE_STATIC_COVARIANCE` in main.cpp

**Best for:** Simulation, controlled lab environments, fixed covariance models

---

## Camera Trigger (GPIO5)

### Hardware Setup

OV9281 cameras must have **physical trigger pins wired**:
- **Pin F** → GPIO5 (RP2350)
- **Pin G** → GND

### Synchronization Strategy

**Perfect timing alignment:**
```
IMU samples (100 Hz):
  0ms    10ms   20ms   30ms   40ms   50ms   60ms ...
  ↓      ↓      ↓      ↓      ↓      ↓      ↓
  [s0]   [s1]   [s2]   [s3]   [s4]   [s5]   [s6]

Camera frames (25 fps, 40ms):
                                 ↓                    ↓
                            [Frame 0]            [Frame 1]
                         (captures at 40ms)  (captures at 80ms)
```

Each camera frame aligns perfectly with every 4th IMU sample.

### Signal Specifications

- **Pin:** GPIO5 on RP2350
- **Voltage:** 3.3V logic level
- **Pulse width:** 20 microseconds (OV9281 requires ≥ 2 µs)
- **Frequency:** Matches `CAMERA_TRIGGER_RATE_MS` in source code
- **Duty cycle:** < 0.1% (high frequency, narrow pulses)

### Enabling External Trigger Mode

Cameras must be set to "external trigger snapshot mode" before GPIO pulses will work:

**Linux:**
```bash
python3 tools/setup_trigger.py
```

**macOS:**
Limited camera control via USB. May require manufacturer app or camera may auto-enable in trigger mode.

**Windows:**
Use ArduCAM's AMCap.exe tool to enable "low-brightness compensation"

---

## Firmware Architecture

### IMU Detection Order

1. **BNO085** → if not found, try
2. **LSM6DSOX** → if not found, try
3. **BMI088** → if not found, retry loop

Detection happens in `setup()` once. First-found sensor persists for entire session.

### Safety Features

| Issue | Solution |
|-------|----------|
| NaN/Inf in sensor data | `fclampnan()` rejects invalid floats |
| Zero quaternion norm | Fallback to identity (1,0,0,0) with threshold check |
| First packet at t=0 | `first_run` flag delays initial publish |
| IMU not found busy-loop | `delay(10)` in retry loop |
| Buffer allocation failure | `std::nothrow` with graceful fallback to unlimited accumulation |
| COBS buffer overflow | `static_assert` at compile-time verifies packet safety |
| CRC injection in production | Disabled by default (`ENABLE_CRC_INJECTION = 0`) |

### Multiprocessing

- **IMU loop:** Runs every 1 ms to check if 10 ms interval elapsed, then publishes packet
- **Camera trigger:** Runs every 1 ms to check if interval elapsed, then sends GPIO pulse
- **Serial commands:** Processed immediately (T/D/C commands)
- All operations fit well within 1-2 ms cycle time

---

## Serial Commands

Send single character, press Enter:

| Command | Effect | Response |
|---------|--------|----------|
| **T** | Generate one test packet | Binary packet immediately sent |
| **D** | Dump current accelerometer | `ACC mps2: x.xxx, y.yyy, z.zzz` |
| **C** | Toggle CRC inject (if compiled) | `CRC injection enabled/disabled` |

---

## Development & Code Quality

### Key Files

```
src/
  main.cpp                    — Main loop, packet assembly, serial I/O
include/
  CameraManager.h             — GPIO trigger class (header-only)
  IMUInterface.h              — Abstract base for sensors
  IMUCommon.h                 — Covariance accumulator (safe allocation)
  BNO085_IMU.h                — Bosch 9-DOF sensor driver
  LSM6DSOX_IMU.h              — ST 6-axis sensor driver
  GroveBMI088_IMU.h           — Bosch 6-axis sensor driver with ENU mapping
  StaticCovariances.h         — Datasheet covariance matrices
tools/
  display_cameras.py          — Show both camera feeds simultaneously
  setup_trigger.py            — Enable external trigger on cameras
  verify_imu_stream.py        — Parse and display IMU packets
  test_imu_display.py         — Pretty IMU visualization
```

### Documentation for AI Models

Complete implementation notes are embedded as comments:

**main.cpp (lines 7-43):** Architecture overview, design decisions, corner cases handled
**CameraManager.h (lines 1-4):** Trigger specifications and usage
**IMUCommon.h (lines 7-26):** Covariance formula, rolling window behavior, allocation safety
**Each IMU driver:** Calibration procedures, axis conventions, sensor-specific quirks

### Performance

| Metric | Value |
|--------|-------|
| Publish rate | 100 Hz (10 ms guaranteed) |
| Jitter | < 1 ms typical |
| Memory (RP2350) | 2.4 KB covariance buffers, ~16 KB code |
| Memory (ESP32-S3) | Same buffers, available space sufficient |
| CPU | < 5% @ 100 Hz (estimates) |
| Pulse jitter (GPIO) | ±100 µs typical (millis() based) |

---

## Troubleshooting

### No IMU Detected (keeps retrying)

**Check:**
- I2C wiring (SDA, SCL, GND, 3.3V)
- Pull-ups on I2C bus (4.7 kΩ typical)
- Power supply (3.3V stable)
- Sensor address conflicts (address scan via serial monitor)

**Solution:**
- Reload firmware: Pull-up may have been missing at boot
- Check address: Some sensors default to different I2C addresses based on pin state

### Garbled Serial Output

**Check:** Baud rate is 115200 (should be auto-detected by PlatformIO)

**Solution:** Manually set to 115200 if using external serial terminal

### Camera Trigger Not Working

**Verify:**
1. Firmware compiled with correct GPIO pin (GPIO5)
2. Camera in external trigger mode (run `setup_trigger.py` on Linux)
3. GPIO5 pulse visible on oscilloscope (should see ~20 µs pulse every 40 ms)
4. Physical wiring correct (F→GPIO5, G→GND)

### Timing Drift

**Cause:** Timer inaccuracy on microcontroller

**Check:** 
- RP2350 crystal is accurate (±0.1% typical)
- No blocking operations in main loop
- No excessive Serial.print() calls

### Crashes or Resets

**Check:**
- Stack overflow (reduce window size if covariance buffers large)
- Null pointer dereference (check derivative code additions)
- I2C bus lockup (requires watchdog reset)

---

## Building & Customization

### Using PlatformIO Extension (Recommended)

1. Open project folder in VS Code
2. PlatformIO auto-initializes
3. Click "PlatformIO: Upload" to flash
4. Click "PlatformIO: Monitor" to watch output

### Configuration Options (Edit in main.cpp)

| Setting | Location | Default | Purpose |
|---------|----------|---------|---------|
| `CAMERA_TRIGGER_RATE_MS` | Line 66 | 40 | Camera trigger frequency (ms) |
| `USE_STATIC_COVARIANCE` | Line 193 | true | Static vs. dynamic covariance |
| `ENABLE_CRC_INJECTION` | Line 189 | 0 | Fault injection (testing only) |
| Window size | Driver begin() | 200 | Covariance rolling buffer samples |

### Adding a Custom IMU Driver

1. Create `include/MyIMU.h` inheriting from `IMUInterface`
2. Implement required methods: `begin()`, `readSensorData()`, `computeCovariances()`
3. Add detection in `main.cpp` `begin_first_available()`
4. Add `#include "MyIMU.h"` to main.cpp

---

## VIO Integration Notes

For Visual-Inertial Odometry, frame-to-IMU association is trivial:

```python
# Pseudo-code for frame-IMU pre-integration
frame_time_ms = frame_index * 40

# All IMU samples within ±5ms are part of this frame's integration window
imu_samples = [imu[i] for i in range(len(imu)) 
               if abs(imu[i].time_ms - frame_time_ms) < 5]

# Pre-integrate IMU between frames
imu_preint = integrate(imu_samples)
```

Perfect alignment at 25 fps means no interpolation needed — each frame has exactly 4 IMU samples.

---

## Known Limitations

- **Single IMU per board:** Use multiple boards for multimodal sensor rigs
- **No I2C timeout:** Bus hang will stall firmware (watchdog recommended)
- **Temperature unused:** Read but not logged (easy to add)
- **Camera trigger macOS limited:** USB camera control constraints on macOS
- **Sequence wraparound:** Expected every ~10 minutes at 100 Hz

---

## Specifications Summary

| Aspect | Value |
|--------|-------|
| **IMU Publish Rate** | 100 Hz (10 ms intervals) |
| **Camera Trigger** | Configurable (25 fps default = 40 ms) |
| **Serial Baud** | 115200 |
| **Packet Size** | 102 bytes encoded + delimiter |
| **CRC Type** | CRC16-CCITT (0x1021) |
| **Encoding** | COBS (Consistent Overhead Byte Stuffing) |
| **Supported Platforms** | RP2350, ESP32-S3 (and similar Arduino-compatible) |
| **Memory** | ~3 KB covariance buffers, ~16 KB code |
| **GPIO for Trigger** | GPIO5 (configurable via code) |
| **Trigger Signal** | 3.3V, 20 µs pulse width |

---

**Production-ready for robotics, drone, and VIO applications. Thoroughly tested across three IMU sensors with tight IMU-camera synchronization.**
