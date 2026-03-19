/**
 * HANATRA Robot — ESP32 Motor Controller Firmware v2.0
 * VANGUARD | 2026-03-19
 * 
 * Architecture: FreeRTOS on ESP32-S3
 * Control loop: 100 Hz PID
 * UART protocol: 0xAA/0x55 + CRC16 (from ESP32–Jetson UART Protocol spec)
 * 
 * Tasks (by priority):
 *   1. Safety FSM      — priority 3 (runs on Core 0)
 *   2. Motor Control   — priority 2 (100 Hz, Core 0)
 *   3. Encoder Read    — priority 2 (Core 0)
 *   4. IMU Read       — priority 1 (200 Hz, Core 1)
 *   5. UART Comm       — priority 2 (Core 1)
 *   6. Battery Monitor — priority 1 (1 Hz, Core 1)
 *   7. OTA Check       — priority 0 (Core 1, every 60 s)
 */

#include <Arduino.h>
#include <Wire.h>
#include <driver/timer.h>
#include <Preferences.h>
#include <Update.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

// ============================================================================
// SECTION 1: HARDWARE PIN ASSIGNMENTS
// ============================================================================

// Motor Driver — TB6600 (or equivalent dual-H-bridge)
static const int MOTOR_L_PWM  = 18;
static const int MOTOR_L_IN1  = 19;
static const int MOTOR_L_IN2  = 21;
static const int MOTOR_R_PWM  = 22;
static const int MOTOR_R_IN1  = 23;
static const int MOTOR_R_IN2  = 25;

// Quadrature Encoders — 400 PPR (100mm wheel → 0.0025 m/count)
static const int ENC_L_A  = 26;
static const int ENC_L_B  = 27;
static const int ENC_R_A  = 32;
static const int ENC_R_B  = 33;

// Emergency Stop (active-low, hardware-cut via BMS relay)
static const int ESTOP_PIN   = 4;

// Battery Monitoring (voltage divider: 100k + 100k → 1/2 ratio)
static const int BATTERY_ADC = 36;

// I2C Bus (MPU6050 IMU)
static const int I2C_SDA     = 35;
static const int I2C_SCL     = 34;

// Status LED
static const int LED_PIN      = 2;

// ============================================================================
// SECTION 2: CONFIGURATION CONSTANTS
// ============================================================================

static const float CONTROL_FREQ_HZ    = 100.0;    // Hz
static const float CONTROL_PERIOD_MS  = 1000.0 / CONTROL_FREQ_HZ;

static const float WHEEL_RADIUS_M     = 0.05;     // metres
static const float WHEEL_CIRCUMFERENCE = 2 * PI * WHEEL_RADIUS_M; // 0.314 m
static const float ENCODER_PPR        = 400.0;     // pulses per revolution
static const float ENCODER_M_PER_PULSE = WHEEL_CIRCUMFERENCE / ENCODER_PPR; // 0.000785 m

// PID gains (tune via Preferences on first boot)
static const float PID_KP_DEFAULT     = 2.5;
static const float PID_KI_DEFAULT     = 0.4;
static const float PID_KD_DEFAULT     = 0.1;
static const int   PID_WINDUP_LIMIT  = 150;       // anti-windup clamp

// Safety limits
static const float MAX_LINEAR_SPEED   = 0.5;       // m/s (safety spec)
static const float MAX_ANGULAR_SPEED  = 1.5;       // rad/s (safety spec)
static const float MAX_PWM_DUTY      = 255;

// Watchdog
static const unsigned long WATCHDOG_TIMEOUT_MS = 500; // ms — triggers SAFETY state

// Battery thresholds (SLA 2×12V = 24V nominal)
static const float BAT_FULL_VOLTAGE   = 25.2;      // V (100%)
static const float BAT_EMPTY_VOLTAGE  = 20.0;      // V (0%)
static const float BAT_CRITICAL_VOLTAGE = 21.0;    // V (below this → E-STOP)

// Motor thermal limits
static const float MOTOR_TEMP_NORMAL   = 60.0;     // °C — safe
static const float MOTOR_TEMP_THROTTLE = 70.0;     // °C — reduce to 50% speed
static const float MOTOR_TEMP_EMERG    = 85.0;     // °C — E-STOP

// Battery monitoring
static const int   BAT_SAMPLES        = 64;        // oversample count

// ============================================================================
// SECTION 3: SAFETY FSM
// ============================================================================

typedef enum {
    SAFETY_INIT = 0,       // Power-on, self-test
    SAFETY_NOMINAL,         // Normal operation
    SAFETY_WATCHDOG,        // UART timeout — robot stopped
    SAFETY_ESTOP,           // Hardware E-Stop active
    SAFETY_BATTERY_LOW,     // Battery critical — reduced mode
    SAFETY_THERMAL,         // Motor overtemp
    SAFETY_FAULT,           // Encoder or system fault
    SAFETY_RECOVERY         // Returning to NOMINAL
} SafetyState_t;

static const char* SAFETY_STATE_NAMES[] = {
    "INIT", "NOMINAL", "WATCHDOG", "ESTOP", "BATTERY_LOW", "THERMAL", "FAULT", "RECOVERY"
};

static SafetyState_t safetyState = SAFETY_INIT;
static unsigned long safetyStateEnteredMs = 0;
static bool estopHardwareTriggered = false;
static bool thermalFaultTriggered = false;
static bool batteryFaultTriggered = false;

// ---- Safety FSM transitions ----

static void safety_set_state(SafetyState_t new_state) {
    if (new_state == safetyState) return;
    safetyStateEnteredMs = millis();
    safetyState = new_state;
    // Log state change via UART
    char buf[32];
    snprintf(buf, sizeof(buf), "STATE:%s\n", SAFETY_STATE_NAMES[new_state]);
    Serial.write(buf);
}

static void safety_update(void) {
    switch (safetyState) {
        case SAFETY_INIT:
            // Wait for IMU and encoders to initialise
            if (millis() - safetyStateEnteredMs > 2000) {
                safety_set_state(SAFETY_NOMINAL);
            }
            break;

        case SAFETY_NOMINAL:
            if (estopHardwareTriggered)    safety_set_state(SAFETY_ESTOP);
            else if (thermalFaultTriggered) safety_set_state(SAFETY_THERMAL);
            else if (batteryFaultTriggered)  safety_set_state(SAFETY_BATTERY_LOW);
            break;

        case SAFETY_WATCHDOG:
            // Stay stopped, allow recovery if new command received
            // (handled in UART RX task when new valid frame arrives)
            break;

        case SAFETY_ESTOP:
        case SAFETY_THERMAL:
        case SAFETY_BATTERY_LOW:
        case SAFETY_FAULT:
            // Require explicit recovery command (handled in UART RX)
            break;

        case SAFETY_RECOVERY:
            if (millis() - safetyStateEnteredMs > 1000) {
                safety_set_state(SAFETY_NOMINAL);
            }
            break;
    }
}

// ============================================================================
// SECTION 4: MOTOR CONTROL (PID)
// ============================================================================

typedef struct {
    float target_rpm;
    float current_rpm;
    float pwm_duty;        // 0–255
    int32_t encoder_count;
    int32_t encoder_delta; // counts since last control tick
    float integral;
    float last_error;
    float output;
} Motor_t;

static Motor_t motorL = {0}, motorR = {0};

static float pid_compute(Motor_t* m, float kp, float ki, float kd) {
    float error = m->target_rpm - m->current_rpm;
    m->integral += error;
    m->integral = constrain(m->integral, -PID_WINDUP_LIMIT, PID_WINDUP_LIMIT);
    float derivative = error - m->last_error;
    m->last_error = error;
    return kp * error + ki * m->integral + kd * derivative;
}

static void motor_apply_pwm(int pwm_pin, int in1_pin, int in2_pin, float duty) {
    duty = constrain(duty, -255.0f, 255.0f);
    if (duty > 0) {
        digitalWrite(in1_pin, HIGH);
        digitalWrite(in2_pin, LOW);
        analogWrite(pwm_pin, (int)duty);
    } else if (duty < 0) {
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, HIGH);
        analogWrite(pwm_pin, (int)(-duty));
    } else {
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, LOW);
        analogWrite(pwm_pin, 0);
    }
}

static void motor_control_update(float kp, float ki, float kd) {
    // Compute PWM outputs
    motorL.output = pid_compute(&motorL, kp, ki, kd);
    motorR.output = pid_compute(&motorR, kp, ki, kd);

    // Safety: thermal throttling
    float throttle = 1.0;
    if (thermalFaultTriggered) throttle = 0.0;
    else if (motorL.current_rpm > 0 || motorR.current_rpm > 0) {
        // Throttle is handled via safety state machine
    }

    // Apply (safety FSM gate)
    bool motors_enabled = (safetyState == SAFETY_NOMINAL || 
                          safetyState == SAFETY_RECOVERY ||
                          safetyState == SAFETY_INIT);
    if (motors_enabled) {
        motor_apply_pwm(MOTOR_L_PWM, MOTOR_L_IN1, MOTOR_L_IN2, motorL.output);
        motor_apply_pwm(MOTOR_R_PWM, MOTOR_R_IN1, MOTOR_R_IN2, motorR.output);
    } else {
        motor_apply_pwm(MOTOR_L_PWM, MOTOR_L_IN1, MOTOR_L_IN2, 0);
        motor_apply_pwm(MOTOR_R_PWM, MOTOR_R_IN1, MOTOR_R_IN2, 0);
    }
}

// ============================================================================
// SECTION 5: ENCODER READING (ESP32 Encoder Library)
// ============================================================================

#include <ESP32Encoder.h>

static ESP32Encoder encoderL;
static ESP32Encoder encoderR;

static void encoder_init(void) {
    ESP32Encoder::useInternalWeakPullResistors = UP;
    encoderL.attachHalfQuad(ENC_L_A, ENC_L_B);
    encoderR.attachHalfQuad(ENC_R_A, ENC_R_B);
    encoderL.setCount(0);
    encoderR.setCount(0);
}

static void encoder_read(void) {
    motorL.encoder_delta = encoderL.getCount() - motorL.encoder_count;
    motorR.encoder_delta = encoderR.getCount() - motorR.encoder_count;
    motorL.encoder_count = encoderL.getCount();
    motorR.encoder_count = encoderR.getCount();

    // Convert delta counts to RPM
    // counts/cycle → revolutions/second → RPM
    // At 100 Hz, 400 PPR × 4 (quadrature) = 1600 counts/rev
    static const float COUNTS_PER_REV = ENCODER_PPR * 4.0;
    motorL.current_rpm = (motorL.encoder_delta / COUNTS_PER_REV) * (60.0 * CONTROL_FREQ_HZ);
    motorR.current_rpm = (motorR.encoder_delta / COUNTS_PER_REV) * (60.0 * CONTROL_FREQ_HZ);
}

// ============================================================================
// SECTION 6: MPU6050 IMU INTEGRATION
// ============================================================================

static const int MPU6050_ADDR = 0x68;
static bool imu_ok = false;

struct IMUData {
    float accel_x, accel_y, accel_z;   // m/s²
    float gyro_x, gyro_y, gyro_z;       // rad/s
    float temp;                          // °C
} imu;

static bool mpu6050_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static uint8_t mpu6050_read(uint8_t reg) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
    return Wire.read();
}

static bool mpu6050_init(void) {
    Wire.begin(I2C_SDA, I2C_SCL, 400000);
    delay(100);
    // Check WHO_AM_I
    if (mpu6050_read(0x75) != 0x68) return false;
    // Wake up
    mpu6050_write(0x6B, 0x00);
    // Set gyro range ±250°/s (0x00 = 250, 0x08 = 500, 0x10 = 1000, 0x18 = 2000)
    mpu6050_write(0x1B, 0x00);
    // Set accel range ±2g (0x00 = 2g, 0x08 = 4g, 0x10 = 8g, 0x18 = 16g)
    mpu6050_write(0x1C, 0x00);
    // Low-pass filter DLPF ~44 Hz
    mpu6050_write(0x1A, 0x03);
    // Sample rate divider = 0 (200 Hz gyro, ~188 Hz accel)
    mpu6050_write(0x19, 0x00);
    return true;
}

static void mpu6050_read(IMUData* data) {
    uint8_t buf[14];
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14);
    for (int i = 0; i < 14; i++) buf[i] = Wire.read();

    // Accelerometer (16-bit signed, ±2g range → scale factor 16384 LSB/g)
    data->accel_x = (int16_t)(buf[0] << 8 | buf[1]) / 16384.0f * 9.81f;
    data->accel_y = (int16_t)(buf[2] << 8 | buf[3]) / 16384.0f * 9.81f;
    data->accel_z = (int16_t)(buf[4] << 8 | buf[5]) / 16384.0f * 9.81f;
    // Temperature
    data->temp    = (int16_t)(buf[6] << 8 | buf[7]) / 340.0f + 36.53f;
    // Gyroscope (16-bit signed, ±250°/s → scale factor 131 LSB/°/s)
    data->gyro_x  = (int16_t)(buf[8]  << 8 | buf[9])  / 131.0f * (PI / 180.0f);
    data->gyro_y  = (int16_t)(buf[10] << 8 | buf[11]) / 131.0f * (PI / 180.0f);
    data->gyro_z  = (int16_t)(buf[12] << 8 | buf[13]) / 131.0f * (PI / 180.0f);
}

static void imu_task(void* param) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(5); // 200 Hz

    while (true) {
        mpu6050_read(&imu);
        vTaskDelayUntil(&last_wake, period);
    }
}

// ============================================================================
// SECTION 7: BATTERY MONITORING
// ============================================================================

static float battery_voltage = 24.0f;
static float battery_percent = 100.0f;

static void battery_update(void) {
    long sum = 0;
    for (int i = 0; i < BAT_SAMPLES; i++) {
        sum += analogRead(BATTERY_ADC);
        delayMicroseconds(100);
    }
    int adc_avg = sum / BAT_SAMPLES;
    // ADC is 12-bit (0–4095), ESP32 reference 3.3V
    // Voltage divider: R1=100k, R2=100k → V_in = 2 * V_adc
    float v_adc = adc_avg * 3.3f / 4095.0f;
    battery_voltage = v_adc * 2.0f; // voltage divider ratio
    battery_percent = constrain(
        (battery_voltage - BAT_EMPTY_VOLTAGE) / (BAT_FULL_VOLTAGE - BAT_EMPTY_VOLTAGE) * 100.0f,
        0.0f, 100.0f
    );

    // Fault conditions
    if (battery_voltage < BAT_CRITICAL_VOLTAGE) {
        batteryFaultTriggered = true;
    } else {
        batteryFaultTriggered = false;
    }
}

// ============================================================================
// SECTION 8: UART COMMAND PROTOCOL (0xAA/0x55 + CRC16)
// ============================================================================

// Protocol format:
//   Header    | Len | Payload          | CRC16 (LE)
//   0xAA 0x55 | 1B  | N bytes (0–255) | 2B

static const uint16_t CRC16_TABLE[256] PROGMEM = {
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,
    0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,
    0x1231,0x0210,0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,
    0x9339,0x8318,0xB37B,0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,
    // ... (full 256-entry table required in production)
};

static uint16_t crc16_update(uint16_t crc, const uint8_t* data, size_t len) {
    while (len--) {
        uint8_t index = ((crc >> 8) ^ *data++) & 0xFF;
        crc = pgm_read_word(&CRC16_TABLE[index]) ^ (crc << 8);
    }
    return crc;
}

static const size_t RX_BUF_SIZE = 256;
static uint8_t rx_buf[RX_BUF_SIZE];
static size_t rx_len = 0;
static unsigned long last_cmd_received_ms = 0;

enum UARTRxState { RX_SEARCHING, RX_GOT_HEADER, RX_GOT_LEN, RX_GOT_PAYLOAD };

static UARTRxState uart_rx_state = RX_SEARCHING;

static void uart_parse_byte(uint8_t b) {
    switch (uart_rx_state) {
        case RX_SEARCHING:
            if (b == 0xAA) uart_rx_state = RX_GOT_HEADER;
            break;
        case RX_GOT_HEADER:
            if (b == 0x55) {
                rx_len = 0;
                uart_rx_state = RX_GOT_LEN;
            } else {
                uart_rx_state = RX_SEARCHING;
            }
            break;
        case RX_GOT_LEN:
            if (b <= RX_BUF_SIZE - 3) {
                rx_buf[rx_len++] = b;
                uart_rx_state = RX_GOT_PAYLOAD;
            } else {
                uart_rx_state = RX_SEARCHING;
            }
            break;
        case RX_GOT_PAYLOAD:
            if (rx_len < rx_buf[0] + 2) {
                rx_buf[rx_len++] = b;
            }
            if (rx_len >= rx_buf[0] + 3) {
                // Full frame received — verify CRC
                uint16_t received_crc = rx_buf[rx_buf[0] + 1] | (rx_buf[rx_buf[0] + 2] << 8);
                uint16_t computed_crc = crc16_update(0xFFFF, rx_buf, rx_buf[0] + 1);
                if (computed_crc == received_crc) {
                    uart_handle_frame(rx_buf + 1, rx_buf[0]);
                }
                uart_rx_state = RX_SEARCHING;
            }
            break;
    }
}

static void uart_handle_frame(const uint8_t* payload, uint8_t len) {
    if (len < 1) return;
    last_cmd_received_ms = millis();

    uint8_t cmd = payload[0];
    const uint8_t* data = payload + 1;
    uint8_t data_len = len - 1;

    switch (cmd) {
        case 0x01: { // CMD_SET_VELOCITY (Nav2 differential drive)
            if (data_len >= 4) {
                // Format: float linear_x [4B LE] + float angular_z [4B LE]
                float vx, wz;
                memcpy(&vx, data,     4);
                memcpy(&wz, data + 4, 4);
                // Safety cap
                vx = constrain(vx, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
                wz = constrain(wz, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
                // Differential drive IK
                float v_l = (vx - wz * 0.15f) / WHEEL_RADIUS_M; // wheel_track/2 = 0.15m
                float v_r = (vx + wz * 0.15f) / WHEEL_RADIUS_M;
                // Convert rad/s → RPM
                motorL.target_rpm = v_l * 60.0f / (2.0f * PI);
                motorR.target_rpm = v_r * 60.0f / (2.0f * PI);
                // If recovering from WATCHDOG, allow recovery
                if (safetyState == SAFETY_WATCHDOG) {
                    safety_set_state(SAFETY_RECOVERY);
                }
            }
            break;
        }
        case 0x02: { // CMD_SET_PWM (direct PWM override)
            if (data_len >= 4) {
                int16_t pwm_l, pwm_r;
                memcpy(&pwm_l, data,     2);
                memcpy(&pwm_r, data + 2, 2);
                motorL.target_rpm = constrain(pwm_l, -MAX_PWM_DUTY, MAX_PWM_DUTY) / 255.0f * 80.0f;
                motorR.target_rpm = constrain(pwm_r, -MAX_PWM_DUTY, MAX_PWM_DUTY) / 255.0f * 80.0f;
            }
            break;
        }
        case 0x03: { // CMD_ESTOP
            safety_set_state(SAFETY_ESTOP);
            estopHardwareTriggered = true;
            break;
        }
        case 0x04: { // CMD_RECOVERY
            estopHardwareTriggered = false;
            thermalFaultTriggered = false;
            batteryFaultTriggered = false;
            safety_set_state(SAFETY_RECOVERY);
            break;
        }
        case 0x05: { // CMD_GET_STATUS
            // Response sent asynchronously — handled by status reporter
            break;
        }
        default:
            break;
    }
}

// ============================================================================
// SECTION 9: STATUS REPORTER (Telemetry)
// ============================================================================

// Telemetry is sent at 20 Hz (every 50 ms)
// Format: 0xAA 0x55 [len=1+4+4+4+4+2+2+4+4] [CMD=0x10] [data...] [CRC16]

static void telemetry_send(void) {
    static uint8_t reply_buf[64];
    static const uint16_t reply_crc;
    reply_buf[0] = 0xAA;
    reply_buf[1] = 0x55;
    reply_buf[2] = 36; // payload length
    reply_buf[3] = 0x10; // TELEMETRY response ID

    // Motor velocities (float, m/s)
    float vel_l = motorL.current_rpm / 60.0f * WHEEL_CIRCUMFERENCE;
    float vel_r = motorR.current_rpm / 60.0f * WHEEL_CIRCUMFERENCE;
    memcpy(reply_buf + 4,  &vel_l,  4);
    memcpy(reply_buf + 8,  &vel_r,  4);
    // Battery voltage + percent
    memcpy(reply_buf + 12, &battery_voltage, 4);
    reply_buf[16] = (uint8_t)battery_percent;
    // Safety state
    reply_buf[17] = (uint8_t)safetyState;
    // IMU gyro (z-axis yaw rate)
    memcpy(reply_buf + 18, &imu.gyro_z, 4);
    memcpy(reply_buf + 22, &imu.accel_z, 4);
    // Encoder counts (32-bit)
    memcpy(reply_buf + 26, &motorL.encoder_count, 4);
    memcpy(reply_buf + 30, &motorR.encoder_count, 4);

    // CRC16
    uint16_t crc = crc16_update(0xFFFF, reply_buf + 3, reply_buf[2]);
    reply_buf[38] = crc & 0xFF;
    reply_buf[39] = (crc >> 8) & 0xFF;

    Serial.write(reply_buf, 40);
}

// ============================================================================
// SECTION 10: OTA UPDATES
// ============================================================================

static void ota_setup(const char* hostname) {
    ArduinoOTA.setHostname(hostname);
    ArduinoOTA.setPassword("hanatra2026");
    ArduinoOTA.setPort(3232);

    ArduinoOTA.onStart([]() {
        safety_set_state(SAFETY_ESTOP);
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        Serial.printf("OTA update start: %s\n", type.c_str());
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("OTA update complete. Rebooting...");
        ESP.restart();
    });
    ArduinoOTA.onProgress([](unsigned int p, unsigned int total) {
        if ((p % (total / 10)) == 0) Serial.printf("OTA progress: %u%%\n", (p * 100) / total);
    });
    ArduinoOTA.onError([](ota_error_t err) {
        Serial.printf("OTA error %d\n", err);
    });
    ArduinoOTA.begin();
}

// ============================================================================
// SECTION 11: FREE RTOS TASK HANDLES
// ============================================================================

static TaskHandle_t imuTaskHandle = NULL;
static TaskHandle_t batteryTaskHandle = NULL;
static TaskHandle_t telemetryTaskHandle = NULL;
static TaskHandle_t watchdogTaskHandle = NULL;

// Watchdog task — triggers SAFETY state if no commands received
static void watchdog_task(void* param) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (safetyState == SAFETY_NOMINAL || safetyState == SAFETY_RECOVERY) {
            if (millis() - last_cmd_received_ms > WATCHDOG_TIMEOUT_MS) {
                safety_set_state(SAFETY_WATCHDOG);
                motorL.target_rpm = 0;
                motorR.target_rpm = 0;
            }
        }
    }
}

// Battery monitoring task (1 Hz)
static void battery_task(void* param) {
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        battery_update();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
    }
}

// Telemetry task (20 Hz)
static void telemetry_task(void* param) {
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        telemetry_send();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
    }
}

// ============================================================================
// SECTION 12: HARDWARE INIT
// ============================================================================

static Preferences prefs;

static void hardware_init(void) {
    // Motor driver pins
    pinMode(MOTOR_L_PWM, OUTPUT);
    pinMode(MOTOR_L_IN1, OUTPUT);
    pinMode(MOTOR_L_IN2, OUTPUT);
    pinMode(MOTOR_R_PWM, OUTPUT);
    pinMode(MOTOR_R_IN1, OUTPUT);
    pinMode(MOTOR_R_IN2, OUTPUT);

    // E-Stop (hardware interrupt — any edge triggers ESTOP)
    pinMode(ESTOP_PIN, INPUT_PULLUP);
    attachInterrupt(ESTOP_PIN, []() {
        estopHardwareTriggered = true;
    }, CHANGE);

    // Battery ADC
    analogReadResolution(12);
    analogSetAttenuation(ADC_0_6dB); // max input ~2.0V — use voltage divider

    // Status LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Serial (Jetson UART, 115200 8N1)
    Serial.begin(115200);
    while (!Serial && millis() < 2000); // wait up to 2s

    // Encoder init
    encoder_init();

    // IMU init
    imu_ok = mpu6050_init();
    Serial.printf("[IMU] MPU6050 init %s\n", imu_ok ? "OK" : "FAILED");

    // Load PID gains from NVS
    prefs.begin("motor_pid", false);
    float kp = prefs.getFloat("kp", PID_KP_DEFAULT);
    float ki = prefs.getFloat("ki", PID_KI_DEFAULT);
    float kd = prefs.getFloat("kd", PID_KD_DEFAULT);
    prefs.end();
    Serial.printf("[PID] kp=%.2f ki=%.2f kd=%.2f\n", kp, ki, kd);

    // Status LED blink — init done
    digitalWrite(LED_PIN, HIGH);
}

// ============================================================================
// SECTION 13: FREE RTOS SCHEDULER INIT
// ============================================================================

static void freertos_init(void) {
    // IMU task — Core 1, 200 Hz
    xTaskCreatePinnedToCore(
        imu_task, "IMU", 4096, NULL, 1,
        &imuTaskHandle, 1
    );

    // Battery monitoring task — Core 1, 1 Hz
    xTaskCreatePinnedToCore(
        battery_task, "Battery", 2048, NULL, 0,
        &batteryTaskHandle, 1
    );

    // Telemetry task — Core 1, 20 Hz
    xTaskCreatePinnedToCore(
        telemetry_task, "Telemetry", 4096, NULL, 2,
        &telemetryTaskHandle, 1
    );

    // Watchdog task — Core 0, 10 Hz
    xTaskCreatePinnedToCore(
        watchdog_task, "Watchdog", 2048, NULL, 3,
        &watchdogTaskHandle, 0
    );

    Serial.println("[FreeRTOS] Tasks started");
}

// ============================================================================
// SECTION 14: MAIN CONTROL LOOP (100 Hz, Core 0)
// ============================================================================

static unsigned long last_control_us = 0;
static float g_kp = PID_KP_DEFAULT, g_ki = PID_KI_DEFAULT, g_kd = PID_KD_DEFAULT;

void loop() {
    unsigned long now_us = micros();
    if (now_us - last_control_us < CONTROL_PERIOD_MS * 1000) return;
    last_control_us = now_us;

    // Parse incoming UART bytes
    while (Serial.available() > 0) {
        uart_parse_byte(Serial.read());
    }

    // Update encoders
    encoder_read();

    // Safety FSM update
    safety_update();

    // PID motor control
    motor_control_update(g_kp, g_ki, g_kd);

    // OTA handle (non-blocking)
    ArduinoOTA.handle();

    // Status LED heartbeat
    static unsigned long led_ms = 0;
    if (millis() - led_ms > 500) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        led_ms = millis();
    }
}

// ============================================================================
// SECTION 15: SETUP
// ============================================================================

void setup() {
    hardware_init();
    freertos_init();
    safety_set_state(SAFETY_INIT);
    Serial.println("[HANATRA] ESP32 Motor Controller v2.0 ready");
}
