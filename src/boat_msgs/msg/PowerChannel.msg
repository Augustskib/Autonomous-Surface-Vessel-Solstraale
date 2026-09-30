# One INA226 channel, part of PowerStatus.

# Channel name from power_monitor's config, e.g. "motor_left", "battery".
string name

float64 voltage_v
float64 current_a
float64 power_w

# False if the I2C read failed. Readings are 0.0 in that case.
bool ok
