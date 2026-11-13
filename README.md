# CanSat Flight Computer

Arduino Nano-based CanSat avionics system for flight data acquisition, telemetry, and recovery support. The firmware combines barometric altitude sensing, inertial measurements, local data logging, and LoRa communications into a single compact flight computer designed for competitive CanSat missions.

## Overview

This repository contains the flight software, hardware references, and supporting documentation for a CanSat built around an Arduino Nano (ATmega328P). The system is designed to:

- measure atmospheric pressure and altitude using a BMP280 sensor
- monitor acceleration and angular rates with an MPU6050 inertial measurement unit
- store flight data locally on a microSD card
- transmit telemetry over LoRa using the RYLR998 radio module
- maintain mission state and recovery data across resets using EEPROM
- detect launch, deployment, descent, and impact conditions
- provide a simple command interface for ground control and field testing

## Key Features

- Real-time sensor sampling at 10 Hz
- 1 Hz telemetry stream for mission monitoring
- Flight-state machine with launch, ascent, deployment, descent, and impact detection
- EEPROM-backed checkpointing to preserve mission state across resets
- Hardware watchdog for recovery from hangs or faults
- Battery voltage monitoring
- Local SD logging for post-flight analysis
- Audible status indication via buzzer and LED indicator

## System Architecture

### Hardware Components

- Controller: Arduino Nano (ATmega328P, CH340-compatible clone)
- Pressure/temperature sensor: BMP280
- IMU: MPU6050
- Storage: microSD card
- Telemetry radio: RYLR998 LoRa module
- Indicators: buzzer and status LED
- Power monitoring: battery divider on analog input A0

### Main Firmware Functions

The flight logic is implemented in [firmware/main.c](firmware/main.c) and includes:

- sensor initialization and calibration
- state-based mission logic
- telemetry packet creation and LoRa transmission
- SD file creation and periodic logging
- EEPROM save/restore routines
- command handling for ground station interaction

## Flight States

The software uses the following mission states:

- BOOT
- TEST
- LAUNCH_PAD
- ASCENT
- DEPLOY
- DESCENT
- AEROBRAKE
- IMPACT

State transitions are driven by altitude, acceleration, descent rate, and time conditions. EEPROM persistence is used to recover important mission state after unexpected resets.

## Ground Commands

The system accepts both serial and LoRa commands. Supported commands include:

- `CX,ON` — enable telemetry transmission
- `CX,OFF` — disable telemetry transmission
- `CAL` — re-zero altitude from the launch pad
- `RST` — start a new mission and reset telemetry/logging state
- `PING` — query the current system state

Every command responds with either `ACK,<cmd>` or `NAK,<cmd>`.

## Startup and Launch Procedure

1. Power on the CanSat and wait for the startup beep sequence.
2. Send `RST` to clear any stale mission state.
3. After clearance, send `CX,ON` to begin telemetry.
4. Monitor telemetry during ascent and descent.
5. After landing, the buzzer and LED provide a repeated recovery signal.

## Repository Structure

```text
.
├── README.md
├── documentation/
│   ├── CanSat_037_Handbook.pdf
│   └── CANSAT_Datatsheet.pdf
├── firmware/
│   └── main.c
├── hardware/
│   ├── pcb/
│   └── schematic/
├── images/
├── testing/
│   ├── logging_tests/
│   └── telemetry_tests/
└── .gitignore
```

## Development and Testing

### Arduino Setup

Use the Arduino IDE with the following configuration:

- Board: `Arduino Nano`
- Processor: `ATmega328P`
- Port: connected Nano serial interface

Required libraries include the standard Arduino core plus:

- `SPI`
- `SD`
- `Wire`
- `SoftwareSerial`
- `EEPROM`

### Validation Data

Sample data files for logging and telemetry validation can be found in:

- [testing/logging_tests](testing/logging_tests)
- [testing/telemetry_tests](testing/telemetry_tests)

These files are useful for verifying expected output structure and telemetry formatting during development and post-flight review.

## Hardware and Design References

- Hardware schematic and PCB resources are available in [hardware](hardware)
- Mission and reference documentation are in [documentation](documentation)
- Visual assets for the build are in [images](images)

## Notes

This project is tailored to a high-altitude CanSat competition environment and emphasizes robust mission continuity, field operation simplicity, and post-flight analysis. The software is intentionally conservative in its persistence strategy and includes fault-tolerant features appropriate for flight use.
