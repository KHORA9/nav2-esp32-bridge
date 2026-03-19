# Hardware Test Procedures

These tests validate the full Nav2 → ESP32 → motor command chain.

**Prerequisites:**
- ESP32 flashed with firmware from `firmware/`
- ROS2 packages built and sourced
- Physical robot wired per `docs/wiring.md`

---

## Equipment Needed

- Oscilloscope or logic analyzer (for latency tests)
- Multimeter
- Tape measure
- Stopwatch
- Laptop running ROS2 Iron

---

## Test Setup

```bash
# Terminal 1: Launch bridge
ros2 launch hanatra_control esp32_bridge.launch.py uart_port:=/dev/ttyUSB0

# Terminal 2: Monitor topics
ros2 topic echo /hanatra/odom
ros2 topic echo /hanatra/motor_status
ros2 topic echo /hanatra/robot_state

# Terminal 3: Send commands
ros2 topic pub /hanatra/cmd_vel geometry_msgs/msg/Twist ...
```

---

## TC-01: Forward Velocity (0.1 m/s)

**Purpose:** Verify encoder feedback accuracy.

**Setup:** Robot on flat floor, 2m marked path.

**Procedure:**
1. Reset encoders (power cycle ESP32)
2. `ros2 topic pub /hanatra/cmd_vel '{linear: {x: 0.1}, angular: {z: 0.0}}' -r 10`
3. Mark start position
4. Run for exactly 10 seconds
5. Measure distance traveled

**Expected:** Distance = 1.0 ± 0.05 m

**Pass criteria:** Distance within ±5% of expected.

---

## TC-02: Max Speed Safety Cap

**Purpose:** Verify 0.5 m/s safety limit is enforced.

**Setup:** Flat floor, 3m clear path.

**Procedure:**
1. `ros2 topic pub /hanatra/cmd_vel '{linear: {x: 1.0}}' -r 10`
2. Measure max speed from odometry or encoder

**Expected:** Robot never exceeds 0.5 m/s.

**Pass criteria:** Speed ≤ 0.5 m/s confirmed by encoder RPM.

---

## TC-03: Pure Rotation

**Purpose:** Verify differential drive turning.

**Setup:** Robot on flat floor, 90° mark on ground.

**Procedure:**
1. `ros2 topic pub /hanatra/cmd_vel '{linear: {x: 0.0}, angular: {z: 0.5}}' -r 10`
2. Time 10 full rotations
3. Calculate observed angular velocity

**Expected:** Angular velocity = 0.5 ± 0.05 rad/s

**Pass criteria:** Angular error < 10%.

---

## TC-04: E-Stop Response Distance

**Purpose:** Measure stopping distance under E-Stop.

**Setup:** Robot moving at 0.3 m/s, 2m clear path ahead.

**Procedure:**
1. Start robot moving forward: `ros2 topic pub /hanatra/cmd_vel '{linear: {x: 0.3}}' -r 10`
2. After 2s, trigger E-Stop: `ros2 topic pub /hanatra/estop std_msgs/msg/Bool 'data: true'`
3. Measure stopping distance

**Expected:** Stopping distance ≤ 0.31 m (from physics: v²/2a = 0.3²/2×0.7 ≈ 0.06m minimum, add margin)

**Pass criteria:** Stop within 0.5 m.

---

## TC-05: Watchdog Timeout

**Purpose:** Verify 500ms UART timeout triggers SAFETY state.

**Setup:** Bridge running, ESP32 connected.

**Procedure:**
1. Monitor `/hanatra/robot_state`
2. Disconnect USB cable
3. Wait 600ms
4. Reconnect and check state

**Expected:** State transitions to WATCHDOG within 500ms.

**Pass criteria:** SAFETY state observed within 600ms of cable disconnect.

---

## TC-06: End-to-End Latency

**Purpose:** Measure Nav2 cmd_vel → wheel response.

**Setup:** Oscilloscope on motor PWM pin.

**Procedure:**
1. Probe ESP32 GPIO 18 (left motor PWM)
2. Send single cmd_vel: `ros2 topic pub /hanatra/cmd_vel ...` (once)
3. Measure time from UART TX to PWM rising edge change

**Expected:** < 50ms total latency.

**Pass criteria:** Measured latency < 50ms.

---

## TC-07: Encoder Fault Detection

**Purpose:** Verify encoder disconnect is detected.

**Setup:** Encoder connected, bridge running.

**Procedure:**
1. Monitor encoder counts via `/hanatra/motor_status`
2. Disconnect encoder A wire from GPIO 26
3. Check robot_state for encoder_fault flag

**Expected:** `encoder_fault: true` within 0.5s.

**Pass criteria:** FAULT state triggered within 1s.

---

## TC-08: Battery Voltage Monitoring

**Purpose:** Verify battery reading accuracy.

**Setup:** Multimeter connected to battery terminals.

**Procedure:**
1. Read multimeter voltage
2. Compare with `/hanatra/battery` voltage reading

**Expected:** Difference < 0.2V

**Pass criteria:** Reading within 0.2V of multimeter.

---

## TC-09: Motor Thermal Throttling

**Purpose:** Verify thermal protection.

**Setup:** Robot stationary, temperature probe on motor winding.

**Procedure:**
1. Run robot at max speed continuously for 10 minutes
2. Monitor `/hanatra/robot_state` motor_temp_c

**Expected:** Temp rises, at >70°C speed throttled, at >85°C E-Stop.

**Pass criteria:** Thermal state machine transitions correct at thresholds.

---

## TC-10: Recovery from Watchdog State

**Purpose:** Verify graceful recovery from WATCHDOG state.

**Setup:** Robot in WATCHDOG state (simulate by stopping UART).

**Procedure:**
1. Send new cmd_vel command
2. Verify robot transitions from WATCHDOG → RECOVERING → NOMINAL

**Expected:** Recovery within 2 seconds of new command.

**Pass criteria:** State returns to NOMINAL automatically.
