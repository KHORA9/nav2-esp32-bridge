# SafetyStatus.msg — Aggregated safety system status
uint8 NOMINAL=0
uint8 WATCHDOG=1
uint8 ESTOP=2
uint8 BATTERY_LOW=3
uint8 THERMAL=4
uint8 FAULT=5
uint8 RECOVERY=6

uint8 state
string description
