# Wiring Reference — HANATRA FORTITUDE

## Pinout Summary

```
ESP32-S3 DevKitC-1

GPIO 4   ←  ESTOP_BTN (active-low via 10k pullup)
GPIO 18  →  MOTOR_L_PWM  (PWM output)
GPIO 19  →  MOTOR_L_IN1  (direction)
GPIO 21  →  MOTOR_L_IN2  (direction)
GPIO 22  →  MOTOR_R_PWM  (PWM output)
GPIO 23  →  MOTOR_R_IN1  (direction)
GPIO 25  →  MOTOR_R_IN2  (direction)
GPIO 26  ←  ENC_L_A      (encoder left A — pullup)
GPIO 27  ←  ENC_L_B      (encoder left B — pullup)
GPIO 32  ←  ENC_R_A      (encoder right A — pullup)
GPIO 33  ←  ENC_R_B      (encoder right B — pullup)
GPIO 34  →  I2C_SCL      (MPU6050 SCL)
GPIO 35  ↔  I2C_SDA      (MPU6050 SDA)
GPIO 36  ←  BATTERY_ADC  (via 100k+100k divider, max 3.3V)
GPIO 2   →  STATUS_LED   (via 330Ω resistor)
```

## Motor Driver Connection (TB6600 or equiv.)

```
ESP32          TB6600 / H-Bridge
────────       ──────────────────
GPIO18  →      PWM-L   (left motor speed)
GPIO19  →      DIR-L1  (left motor direction)
GPIO21  →      DIR-L2  (left motor direction)
GPIO22  →      PWM-R   (right motor speed)
GPIO23  →      DIR-R1  (right motor direction)
GPIO25  →      DIR-R2  (right motor direction)

GND      →     GND
5V/12V  →     VCC (driver supply, see driver spec)
```

## Differential Drive Encoder Wiring

```
Motor Encoder (5V signals)    ESP32 (3.3V tolerant — use pullup to 3.3V)
─────────────────────────     ─────────────────────────────────────────
Encoder L A  →  10kΩ → 3.3V  →  GPIO26
Encoder L B  →  10kΩ → 3.3V  →  GPIO27
Encoder R A  →  10kΩ → 3.3V  →  GPIO32
Encoder R B  →  10kΩ → 3.3V  →  GPIO33
Encoder GND  →  ESP32 GND
Encoder VCC  →  5V (from ESP32 or external)
```

**Note:** The ESP32Encoder library uses internal weak pullups. Use a 10kΩ
pullup to 3.3V on encoder signal lines if encoder is 5V powered.

## MPU6050 IMU Connection

```
MPU6050        ESP32
────────       ──────
VCC      →     3.3V (do NOT use 5V!)
GND      →     GND
SDA      →     GPIO35
SCL      →     GPIO34
```

**Important:** The MPU6050 is 3.3V only. Do NOT connect to 5V VCC.

## Battery Monitoring

```
Battery (24V) ──┬─── 100kΩ ──┬─── GPIO36 ── ESP32 ADC
                │             │
                └─── 100kΩ ──┘
                              │
                             GND
```

**ADC reading:** `voltage = adc_read * 3.3V / 4096 * 2` (voltage divider halves battery voltage)

## E-Stop Button

```
E-STOP BUTTON (NC, closing to GND)
  │
  └──→ GPIO4 (internal pullup enabled)
  │
  └──→ 10kΩ pullup to 3.3V (optional if using INPUT_PULLUP)

When button pressed: GPIO4 goes LOW → ESTOP triggered
```

## USB-UART (ESP32 ↔ Jetson Nano)

```
Jetson Nano / Laptop     ESP32-S3 DevKit
─────────────────────    ────────────────
USB-A (data)     →      USB-C (programming/UART)
  (ttyUSB0 / ttyACM0)
```

**UART settings:** 115200 baud, 8N1, no flow control

## Full Wiring Diagram (ASCII)

```
                    ┌──────────────────┐
                    │   ESP32-S3 DevKit │
                    │   (USB-C for     │
                    │    programming)    │
                    └──────┬───────────┘
                           │ USB-UART (/dev/ttyUSB0)
                           │ 115200 8N1
                    ┌──────▼───────────┐
                    │  Jetson Nano /   │
                    │  Raspberry Pi 4  │
                    │  (ROS2 Iron)     │
                    └──────────────────┘

GPIO18 ────────────→ MOTOR_L_PWM ──→ TB6600 L ──→ Left Motor
GPIO19 ────────────→ DIR_L1
GPIO21 ────────────→ DIR_L2
GPIO22 ────────────→ MOTOR_R_PWM ──→ TB6600 R ──→ Right Motor
GPIO23 ────────────→ DIR_R1
GPIO25 ────────────→ DIR_R2

GPIO26 ←─────────── ENC_L_A ←── Encoder L (A)
GPIO27 ←─────────── ENC_L_B ←── Encoder L (B)
GPIO32 ←─────────── ENC_R_A ←── Encoder R (A)
GPIO33 ←─────────── ENC_R_B ←── Encoder R (B)

GPIO4  ←── ESTOP button (NC to GND)

GPIO34 → I2C_SCL → MPU6050 SCL
GPIO35 ↔ I2C_SDA ↔ MPU6050 SDA

GPIO36 ← Battery ADC (via 100k/100k divider)

GPIO2 → Status LED (+ 330Ω → GND)
```

## Bill of Materials (Electronics)

| Component | Part | Quantity | Notes |
|-----------|------|---------|-------|
| Microcontroller | ESP32-S3 DevKitC-1 | 1 | Dual-core, 240MHz |
| Motor driver | TB6600 or DRV8871 | 1 | 2-channel, 10–40V |
| IMU | MPU6050 | 1 | 6-DOF, I2C |
| Encoders | 400 PPR quadrature | 2 | Optical or magnetic |
| Battery | 2× 12V SLA or 3S LiPo | 1 | 20–25V nominal |
| Resistors | 100kΩ, 1% | 2 | For voltage divider |
| Resistor | 330Ω | 1 | Status LED |
| Wire | 22 AWG | as needed | Signal wiring |
| Wire | 18 AWG | as needed | Motor power |
| E-Stop button | NC, panel mount | 1 | Safety rating |
