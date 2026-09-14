# The four INA226 sensors, each on a different rail.
#
# WHICH SENSOR IS ON WHICH RAIL WAS MEASURED, NOT READ OFF THE SCHEMATIC.
# Two of the addresses were the wrong way round: 0x41 is the processor rail and
# 0x40 is a motor rail, where power.h had claimed the opposite. Pack voltage
# still came out right by luck, but motor A's current was the processor's - a
# wrong number that looks entirely plausible. See firmware/config/power.h.
std_msgs/Header header

# Motor rails, behind the fuse, mushroom switch and relay. Both sit on the same
# node, so with the contactor closed they read the pack; with it open they read
# nothing. Which one is physically motor A is still unproven - it needs a motor
# to spin.
float32 motor_a_volts
float32 motor_a_amps
float32 motor_b_volts
float32 motor_b_amps

float32 fan_volts
float32 fan_amps

# The ESP32 +5 V rail AFTER its buck, and the whole-module draw measured on the
# battery side. The two are different nodes, so volts * amps here is NOT a power
# figure for anything.
float32 esp32_rail_volts
float32 module_amps

# False whenever the pack could not be MEASURED - the contactor is open, so the
# motor rails are dead. That is not the same as a flat battery, and treating it
# as one is what used to latch the robot into e-stop three seconds after boot.
bool pack_valid
