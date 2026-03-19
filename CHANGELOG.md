# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

### Added
- `encoder_ppr` and `encoder_microsteps` ROS2 runtime parameters — no recompile needed for different encoders (ADR-013)
- `use_sim_time` parameter — clean simulation/hardware clock transition (ADR-013)
- Runtime-configurable `wheel_radius` and `wheel_track` parameters (ADR-013)

## [1.0.0] — 2026-03-19 — Initial Release

### Added
- **ESP32 Motor Controller Firmware v2.0**
  - FreeRTOS-based multi-task architecture
  - Safety FSM: BOOT → INIT → NORMAL → SAFETY/FAULT states
  - PID velocity control at 100 Hz
  - Watchdog timer (500ms timeout → SAFETY state)
  - MPU6050 IMU integration at 200 Hz
  - Battery monitoring with voltage divider
  - OTA update support via ArduinoOTA
  - NVS PID gain storage
  - CRC16-Modbus UART protocol

- **ROS2 Bridge Node (hanatra_control)**
  - UART protocol implementation in C++
  - Differential drive IK (cmd_vel → wheel velocities)
  - Odometry publisher (20 Hz)
  - IMU data publisher (200 Hz)
  - Battery state publisher (1 Hz)
  - E-Stop subscriber
  - TF2 odom → base_link broadcaster
  - Simulation mode (works without physical ESP32)

- **Custom Message Types (hanatra_msgs)**
  - RobotState, MotorCommand, MotorStatus, NavStatus
  - Telemetry, SensorFusionState, Diagnostics
  - EStopStatus, SafetyStatus

- **Documentation**
  - Full UART protocol reference
  - Hardware test procedures (10 test cases)
  - Wiring diagram and pinout
  - ROS2 topic reference
  - Quick start guide

### Features
- Differential drive kinematics: 0.5 m/s max, 1.5 rad/s max angular
- Safety limits enforced at firmware level (not just software)
- Simulation mode for testing without hardware
- YAML-configurable UART port and baud rate
- MIT License — open for commercial and academic use

### Known Limitations
- IMU magnetometer not used (MPU6050 is 6-DOF, no mag)
- Battery SoC estimation is voltage-based (not Coulomb counting)
- PID gains must be tuned per motor batch
- No ROS2 simulation without actual Gazebo URDF (see HANATRA FORTITUDE sim workspace)

### Dependencies
- Firmware: PlatformIO + ESP32-Arduino framework
- ROS2: Iron or Humble, rclcpp, geometry_msgs, nav_msgs, sensor_msgs

### Open-Source Position
- **Zero open-source competitors** in ESP32 + ROS2 + Nav2 integration (confirmed via GitHub API, March 2026)
- First-mover in this niche; HANATRA's bridge is the first production-quality open-source solution
