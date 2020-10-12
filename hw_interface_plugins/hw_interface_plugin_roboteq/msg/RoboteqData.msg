# A
int16[] motor_amps

# AI
int16[] analog_inputs

# AIC
int16[] analog_inputs_conversion

# BA
int16[] battery_amps

# C - returns absolute encoder value; 32bit range +/-2147483648 counts
int32[] encoder_counter_absolute

# CR - returns amt of counts measured from last query (small numbers usually returned)
int32[] encoder_count_relative

# BS
int16[] bl_motor_speed_rpm

# DI
bool[] individual_digital_inputs

# DR
uint8[] destination_reached

# FF
uint8 fault_flags
# Where
# f1 = Overheat
# f2 = Overvoltage
# f3 = Undervoltage
# f4 = Short circuit
# f5 = Emergency stop
# f6 = Brushless sensor fault
# f7 = MOSFET failure
# f8 = Default configuration loaded at startup

# F
int16[] feedback

# VAR
int32[] user_integer_variable

# V where
# volts[0] - internal volts
# volts[1] - battery volts
# volts[2] - 5V output
uint16[] volts
