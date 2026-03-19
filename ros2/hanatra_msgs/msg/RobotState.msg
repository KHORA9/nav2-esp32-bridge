# RobotState.msg — High-level robot state
uint8 BOOT=0
uint8 INIT=1
uint8 IDLE=2
uint8 NORMAL=3
uint8 RECOVERING=4
uint8 SAFETY=5
uint8 FAULT=6

uint8 state
float32 battery_voltage
float32 battery_percent
float32 motor_temp_c
bool estop_active
bool encoder_fault
bool imu_fault
bool battery_fault
