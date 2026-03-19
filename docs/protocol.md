# UART Protocol Reference

## Frame Format

Every frame follows this structure:

```
Byte 0:     0xAA           (SOF — Start of Frame)
Byte 1:     Command ID      (see table below)
Byte 2:     Payload length  (0–255)
Byte 3..N:  Payload        (command-specific data)
Byte N+1:   CRC16-LO       (Modbus CRC, little-endian)
Byte N+2:   CRC16-HI
Byte N+3:   0x55           (EOF — End of Frame)
```

**Total frame size = 4 + LEN + 3 = 7 + LEN bytes**

## CRC16-Modbus Calculation

```c
uint16_t crc16_modbus(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
        }
    }
    return crc;
}
```

CRC is computed over **bytes 1 through N+2** (CMD + LEN + payload), then split into LO/HI bytes.

## Command Reference

### 0x01 — MOTOR_CMD (Jetson → ESP32)

Velocity command in mm/s.

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | left_mm_s | int16 LE | Left wheel velocity, mm/s |
| 2 | right_mm_s | int16 LE | Right wheel velocity, mm/s |

**Range:** ±500 mm/s (±0.5 m/s safety limit enforced by firmware)

Example (forward 0.1 m/s = 100 mm/s):
```
AA 01 04  E8 00 E8 00  5A 23  55
 SOF CMD LEN L   R   CRC      EOF
```

### 0x02 — MOTOR_STATUS (ESP32 → Jetson)

Encoder and RPM feedback.

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | left_counts | int32 LE | Cumulative left encoder counts |
| 4 | right_counts | int32 LE | Cumulative right encoder counts |
| 8 | left_rpm | int16 LE | Left motor RPM |
| 10 | right_rpm | int16 LE | Right motor RPM |

### 0x03 — IMU (ESP32 → Jetson)

MPU6050 raw sensor data.

| Offset | Field | Type | Scale | Description |
|--------|-------|------|-------|-------------|
| 0 | ax | int16 LE | 16384 LSB/g | Accel X |
| 2 | ay | int16 LE | 16384 LSB/g | Accel Y |
| 4 | az | int16 LE | 16384 LSB/g | Accel Z |
| 6 | gx | int16 LE | 131 LSB/°/s | Gyro X |
| 8 | gy | int16 LE | 131 LSB/°/s | Gyro Y |
| 10 | gz | int16 LE | 131 LSB/°/s | Gyro Z |

### 0x04 — ESTOP (bidirectional)

Emergency stop command.

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | reason | uint8 | 0x01=software, 0x02=thermal, 0x04=watchdog, 0x08=battery |

### 0x05 — BATTERY (ESP32 → Jetson)

Battery status.

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | voltage_mv | uint16 LE | Battery voltage in mV |
| 2 | percent | uint8 | State of charge 0–100% |

### 0x06 — DIAGNOSTICS (ESP32 → Jetson)

System diagnostics.

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | motor_temp_c | float32 LE | Motor winding temperature °C |
| 4 | fault_flags | uint8 | Bitfield: bit0=encoder, bit1=IMU, bit2=battery |

### 0x0A — HEARTBEAT (bidirectional)

Link health check.

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | seq | uint8 | Sequence number (wrap-around) |
| 1 | state | uint8 | Safety FSM state |

### 0x10 — TELEMETRY (ESP32 → Jetson, 20 Hz)

Full telemetry burst.

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | vel_l_m_s | float32 LE | Left wheel velocity m/s |
| 4 | vel_r_m_s | float32 LE | Right wheel velocity m/s |
| 8 | battery_v | float32 LE | Battery voltage V |
| 12 | battery_pct | uint8 | SOC % |
| 13 | safety_state | uint8 | Safety FSM state |
| 14 | gyro_z | float32 LE | Yaw rate rad/s |
| 18 | accel_z | float32 LE | Vertical accel m/s² |
| 22 | left_counts | int32 LE | Left encoder cumulative |
| 26 | right_counts | int32 LE | Right encoder cumulative |

## Safety FSM States

| Value | Name | Description |
|-------|------|-------------|
| 0 | INIT | Power-on self-test |
| 1 | NOMINAL | Normal operation |
| 2 | WATCHDOG | UART timeout — motors stopped |
| 3 | ESTOP | E-Stop triggered |
| 4 | BATTERY_LOW | Battery below 21V |
| 5 | THERMAL | Motor > 85°C |
| 6 | FAULT | Requires power cycle |
| 7 | RECOVERY | Returning to NOMINAL |
