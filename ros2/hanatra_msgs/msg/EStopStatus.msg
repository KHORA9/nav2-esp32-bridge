# EStopStatus.msg — E-Stop system status
bool hardware_estop       # Physical button
bool software_estop        # Software-triggered
bool thermal_estop         # Motor overtemp
bool battery_estop         # Low battery
bool watchdog_estop        # UART timeout
bool fault                 # Requires power cycle
