# FlightComp

An ESP32-based flight computer for model rocketry, developed as a prototype for autonomous flight-state detection, telemetry, data logging, and recovery-system control.

The current version is a hand-wired prototype used to validate the avionics architecture and flight-state logic before moving to a custom PCB.

> **Current status:** Ground-test prototype. `GROUND_TEST_MODE` is enabled in the current firmware. This version has not been flight-tested.


<img width="899" height="1599" alt="flicomp" src="https://github.com/user-attachments/assets/acc387cc-feeb-4e26-9269-c50b85d0e3e6" />


---

## Overview

The flight computer continuously collects data from an **MPU6500 IMU** and **BMP280 barometric sensor**, processes the measurements, determines the current flight state, logs data to a microSD card, and transmits telemetry over an **nRF24L01 PA+LNA** radio.

The firmware is built around a state machine rather than treating the rocket as a collection of independent sensor readings.

The current flight states are:

```text
BOOT
  ↓
SENSOR_INIT
  ↓
CALIBRATING
  ↓
PAD_IDLE
  ↓
ARMED
  ↓
BOOST
  ↓
COAST
  ↓
APOGEE
  ↓
DESCENT
  ↓
LANDED
```

An `ERROR_STATE` is available as a safety fallback when critical initialization, sensor, calibration, or watchdog conditions fail.

---

## Features

* ESP32-based flight computer
* MPU6500 accelerometer/IMU
* BMP280 barometric altimeter
* nRF24L01 PA+LNA telemetry
* microSD flight-data logging
* 50 Hz main processing loop
* Sensor calibration and automatic vertical-axis detection
* Altitude, velocity, and acceleration estimation
* Launch detection
* Burnout detection
* Apogee detection using multiple conditions
* Landing detection
* State-specific LED and buzzer feedback
* Pyro output with automatic cutoff
* Watchdog timeout detection
* Ground-test mode with reduced thresholds
* Separate SPI buses for the SD card and other SPI peripherals

---

## System Architecture

```text
                         ┌──────────────────────┐
                         │        ESP32         │
                         │    Flight Computer   │
                         └──────────┬───────────┘
                                    │
             ┌──────────────────────┼──────────────────────┐
             │                      │                      │
             ▼                      ▼                      ▼
       ┌───────────┐          ┌───────────┐         ┌────────────┐
       │  MPU6500  │          │  BMP280   │         │  nRF24L01  │
       │    IMU    │          │  Altitude │         │ Telemetry  │
       └───────────┘          └───────────┘         └─────┬──────┘
                                                           │
                                                           │ RF
                                                           ▼
                                                    Ground Station

             ┌──────────────────────┐
             │      microSD          │
             │   Flight Logging      │
             └──────────────────────┘

             ┌──────────────────────┐
             │ LED / Buzzer / Pyro  │
             │     Outputs          │
             └──────────────────────┘
```

---

## Hardware

### Microcontroller

**ESP32**

The ESP32 handles:

* Sensor acquisition
* Sensor calibration
* Flight-state processing
* Altitude and velocity calculations
* Telemetry generation
* SD-card logging
* State feedback
* Pyro control
* Watchdog monitoring

### Sensors

| Device          | Interface | Purpose                           |
| --------------- | --------- | --------------------------------- |
| MPU6500         | SPI       | Acceleration and motion detection |
| BMP280          | I²C       | Barometric altitude               |
| nRF24L01 PA+LNA | SPI       | Telemetry                         |
| microSD         | SPI       | Flight-data logging               |

### Other peripherals

* Status LED
* Buzzer
* Pyro output

---

## SPI Architecture

One of the problems encountered during development was sharing SPI peripherals without allowing the SD card to interfere with the IMU and radio.

The ESP32 therefore uses two separate SPI buses.

### VSPI

Used by:

* MPU6500
* nRF24L01

```text
SCK  → GPIO 18
MISO → GPIO 19
MOSI → GPIO 23
```

MPU6500:

```text
CS → GPIO 5
```

nRF24L01:

```text
CE  → GPIO 16
CSN → GPIO 17
```

### HSPI

Used exclusively by the microSD card:

```text
SCK  → GPIO 14
MISO → GPIO 35
MOSI → GPIO 13
CS   → GPIO 4
```

This keeps the SD card on a separate SPI bus from the IMU and telemetry radio.

---

## Sensor Calibration

Before entering the normal pad state, the flight computer performs a calibration sequence.

The rocket is expected to remain stationary and vertical during calibration.

The firmware collects **200 samples** from the MPU6500 and BMP280 and calculates:

* Accelerometer bias
* Ground altitude
* Gravity magnitude
* Vertical acceleration axis
* Vertical acceleration sign

The largest accelerometer axis is automatically identified as the rocket's vertical axis.

This allows the firmware to determine whether the rocket is oriented along the X, Y, or Z axis without hard-coding the axis into the flight algorithm.

The system also establishes the initial ground-altitude reference during calibration.

---

## Flight State Machine

### 1. BOOT

Initial system state.

The firmware initializes the hardware and records the boot time.

---

### 2. SENSOR_INIT

The flight computer initializes:

* microSD
* MPU6500
* BMP280
* nRF24L01

If a critical sensor fails to initialize, the system enters `ERROR_STATE`.

---

### 3. CALIBRATING

The system collects sensor samples and determines:

* Accelerometer bias
* Gravity magnitude
* Vertical axis
* Ground altitude

The system remains in this state until calibration succeeds.

---

### 4. PAD_IDLE

The rocket is stationary on the pad.

The firmware monitors acceleration and requires the system to remain still for a defined period before transitioning to `ARMED`.

The current implementation requires:

```text
Minimum pad stillness: 2 seconds
```

The ground reference is also allowed to slowly update while the system remains stationary, providing basic correction for barometric drift.

---

### 5. ARMED

The flight computer is ready to detect launch.

Launch detection requires consecutive acceleration samples above the configured threshold.

The system also monitors movement while armed.

If significant movement is detected while the system is still in the armed state, it returns to `PAD_IDLE`.

This prevents an armed system from remaining armed after being physically disturbed.

---

### 6. BOOST

Launch has been detected.

The system monitors acceleration to determine when motor burnout has occurred.

A timeout is also implemented so the system does not remain indefinitely in the boost state if the expected transition does not occur.

---

### 7. COAST

After burnout, the rocket enters the coast phase.

Apogee detection uses multiple conditions:

* Altitude has stopped increasing
* Velocity indicates descent
* Acceleration indicates descent
* Minimum altitude requirement is satisfied

The conditions are combined into a voting/confirmation system rather than relying on a single sensor threshold.

A coast timeout provides a fallback path if normal apogee detection fails.

---

### 8. APOGEE

Once apogee is confirmed, the system enters the apogee state.

The recovery output is triggered after the configured apogee delay.

The current firmware also includes a backup check when entering descent to ensure that the recovery output has been triggered.

---

### 9. DESCENT

The rocket is descending after recovery deployment.

The flight computer continues collecting and logging telemetry while monitoring for landing conditions.

---

### 10. LANDED

Landing is detected when:

* Altitude is sufficiently close to the ground reference
* Velocity is below the landing threshold
* The conditions remain valid for the required confirmation period

A flight summary is then generated and the remaining log data is flushed to the SD card.

---

### 11. ERROR_STATE

Critical failures transition the system into `ERROR_STATE`.

Examples include:

* Sensor initialization failure
* Failed calibration
* Excessive sensor errors
* Watchdog timeout

The LED and buzzer provide an error indication.

---

## Sensor Processing

The main loop runs at:

```text
50 Hz
```

Sensor measurements are filtered before being used by the flight-state logic.

### Altitude

Barometric altitude is calculated from the BMP280 pressure measurement and filtered using an IIR filter.

### Acceleration

The vertical acceleration component is determined from the calibrated vertical axis and gravity direction.

The filtered acceleration is then used for:

* Launch detection
* Burnout detection
* Apogee detection
* Movement detection
* Maximum acceleration tracking

### Velocity

Velocity is estimated from the change in filtered altitude:

```text
velocity = Δaltitude / Δtime
```

The resulting velocity estimate is filtered before being used by the state machine.

---

## Launch Detection

Launch detection uses consecutive acceleration samples rather than a single threshold crossing.

The flight computer requires:

```text
Acceleration > launch threshold
```

for a configured number of consecutive samples.

In ground-test mode, the thresholds are intentionally reduced so the entire flight sequence can be simulated through controlled movement.

---

## Burnout Detection

Burnout detection monitors the filtered vertical acceleration.

A number of consecutive low-acceleration samples are required before transitioning from `BOOST` to `COAST`.

A minimum boost duration and a boost timeout are also implemented.

---

## Apogee Detection

Apogee detection is designed around the altitude profile of the rocket.

The altitude must stop increasing and begin decreasing, while supporting conditions from acceleration and velocity indicate descent.

Conceptually:

```text
                Peak altitude
                    ▲
                    │
              COAST │
                    │
          ──────────┼──────────
                    │\
                    │ \
                    │  \
                    │   \  DESCENT
                    │    \
                    ▼
                 APOGEE
```

The firmware requires the apogee condition to remain valid for multiple samples before accepting the transition.

This reduces the chance of triggering recovery from a single noisy measurement.

---

## Telemetry

Telemetry is transmitted using the nRF24L01.

The telemetry packet is limited to the nRF24L01's 32-byte payload size.

The current packet contains:

| Field                | Description                            |
| -------------------- | -------------------------------------- |
| Timestamp            | Packet timestamp                       |
| State                | Current flight state                   |
| Altitude             | Filtered altitude above ground         |
| Velocity             | Filtered velocity                      |
| Acceleration         | Filtered vertical acceleration         |
| Maximum altitude     | Maximum recorded altitude              |
| Maximum velocity     | Maximum recorded velocity              |
| Maximum acceleration | Maximum recorded acceleration          |
| Battery              | Reserved for future battery monitoring |
| Checksum             | XOR checksum                           |

The complete packet is currently **31 bytes**.

### Telemetry rate

The transmission rate changes depending on flight state:

| State        |  Rate |
| ------------ | ----: |
| BOOST        | 20 Hz |
| COAST        | 20 Hz |
| APOGEE       | 20 Hz |
| DESCENT      | 20 Hz |
| ARMED        |  5 Hz |
| PAD_IDLE     |  1 Hz |
| LANDED       |  1 Hz |
| Other states |  2 Hz |

---

## Data Logging

Flight data is stored on a microSD card as CSV.

A new filename is automatically generated:

```text
flight000.csv
flight001.csv
flight002.csv
...
```

The logged telemetry contains:

```text
Time(ms)
Type
State
AGL(m)
Velocity(m/s)
Accel(g)
Alt_Raw(m)
Accel_Raw(g)
```

Logging uses a RAM buffer to reduce the amount of blocking SD-card activity.

The firmware deliberately avoids flushing the SD card during the most time-critical flight phases:

```text
BOOST
COAST
APOGEE
```

The buffered data is flushed later.

---

## Ground Test Mode

The firmware includes a dedicated ground-test mode:

```cpp
#define GROUND_TEST_MODE true
```

This allows the flight sequence to be exercised without actually launching the rocket.

Ground-test thresholds are intentionally much lower than the configured flight thresholds.

The firmware provides a simple test sequence:

```text
Keep still for 2 s
       ↓
     ARMED
       ↓
Lift approximately 50 cm
       ↓
     BOOST
       ↓
Hold briefly
       ↓
     COAST
       ↓
Lower approximately 10 cm
       ↓
    APOGEE
       ↓
Continue lowering
       ↓
    DESCENT
       ↓
Place on table
       ↓
    LANDED
```

This allows the complete state machine, telemetry, logging, feedback, and recovery-control logic to be exercised on the bench.

---

## Feedback

The flight computer provides both visual and audible state feedback.

| State                 | LED           | Audio                |
| --------------------- | ------------- | -------------------- |
| Boot / initialization | Slow blink    | Startup tones        |
| Pad idle              | Solid ON      | None                 |
| Armed                 | Rapid flicker | Two beeps            |
| Boost / Coast         | OFF           | None                 |
| Apogee                | ON            | One long beep        |
| Descent               | ON            | None                 |
| Landed                | OFF           | Three beeps          |
| Error                 | Fast blink    | Repeating error tone |

---

## Pyro Control

The firmware includes a dedicated recovery-output control path.

The output is initialized LOW during startup.

The firmware also tracks whether the output has already been triggered to prevent repeated firing.

A configured pulse duration is used, after which the output is automatically returned LOW.

The recovery sequence includes:

1. Apogee detection
2. Configured deployment delay
3. Recovery-output activation
4. Automatic output cutoff
5. Confirmation before entering descent

A secondary check is performed when entering `DESCENT` to detect the unexpected case where the recovery output has not already been triggered.

> **Safety:** This repository documents the firmware architecture and bench-test implementation. The current system is a prototype and should not be considered flight-qualified.

---

## Watchdog

A software watchdog monitors the main loop.

If the loop does not update within the configured timeout, the system enters `ERROR_STATE`.

Current timeout:

```text
1000 ms
```

The firmware also tracks loop overruns to help identify timing problems during testing.

---

## Configuration

Important parameters are centralized near the beginning of the firmware.

Examples include:

```cpp
#define GROUND_TEST_MODE true

#define LOOP_INTERVAL_MS 20
#define SAMPLE_RATE_HZ 50

#define CALIBRATION_SAMPLES 200

#define ACCEL_STILLNESS_THRESHOLD 0.1

#define BOOST_TIMEOUT_MS 15000
#define COAST_TIMEOUT_MS 45000

#define PYRO_FIRE_DURATION 1000
#define PYRO_DELAY_AFTER_APOGEE 200
```

Flight and ground-test thresholds are separated using conditional compilation so that the two operating modes can be tested independently.

---

## Software Structure

The firmware is organized into functional sections:

```text
Configuration
│
├── Hardware configuration
├── Flight thresholds
├── Timing
└── Calibration parameters
│
├── MPU6500 driver
├── BMP280 driver
├── nRF24L01 driver
├── SD-card logging
│
├── Sensor calibration
├── Sensor processing
│
├── Flight state machine
├── Telemetry
├── Pyro control
├── Audio / visual feedback
└── Watchdog
```

The main execution loop follows this sequence:

```text
Read sensors
    ↓
Validate sensor data
    ↓
Update filtered values
    ↓
Run state machine
    ↓
Generate telemetry
    ↓
Update feedback
    ↓
Update pyro control
    ↓
Transmit telemetry
    ↓
Handle logging
```

---

## Current Status

| Feature                           | Status            |
| --------------------------------- | ----------------- |
| ESP32 flight computer             | Prototype         |
| MPU6500 integration               | Implemented       |
| BMP280 integration                | Implemented       |
| Sensor calibration                | Implemented       |
| Automatic vertical-axis detection | Implemented       |
| Flight state machine              | Implemented       |
| Launch detection                  | Implemented       |
| Burnout detection                 | Implemented       |
| Apogee detection                  | Implemented       |
| Landing detection                 | Implemented       |
| nRF24 telemetry                   | Implemented       |
| microSD logging                   | Implemented       |
| LED / buzzer feedback             | Implemented       |
| Watchdog                          | Implemented       |
| Ground-test mode                  | Implemented       |
| Custom flight-computer PCB        | Next iteration    |
| Flight testing                    | Not yet performed |

---

## Development Roadmap

* [x] Build initial ESP32 prototype
* [x] Implement sensor drivers
* [x] Implement calibration
* [x] Develop flight-state machine
* [x] Implement launch detection
* [x] Implement burnout detection
* [x] Implement apogee detection
* [x] Implement landing detection
* [x] Add telemetry
* [x] Add SD-card logging
* [x] Add ground-test mode
* [x] Add watchdog and error handling
* [x] Move the design to a custom PCB
* [ ] PCB fabrication
* [ ] Hardware bring-up
* [ ] Sensor and telemetry validation on PCB
* [ ] Full ground testing
* [ ] Rocket integration
* [ ] Flight testing

---

## Project Context

This prototype was developed as part of the personal avionics work.

The purpose of this version was not simply to make a flight computer that could read sensors. It was to develop and validate the software architecture and flight-state logic before committing the design to custom hardware.

The next iteration moves the system from hand-wired prototype hardware to a dedicated flight-computer PCB.

---

## License

