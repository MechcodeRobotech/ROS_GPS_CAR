# What the two drive motors are being asked to do and what they appear to do.
std_msgs/Header header

# Commanded duty, -1.0 to 1.0. Sign is direction.
float32 pwm_left
float32 pwm_right

# Wheel speed derived from the body estimate and the yaw rate. This robot has no
# encoders, so these are computed, not measured.
float32 wheel_rpm_left
float32 wheel_rpm_right
float32 wheel_mps_left
float32 wheel_mps_right

# Shaft speed from the back-EMF model: (V_bus - V_diode) * duty - I * R, over Ke.
# It uses the MEASURED diode drops, which are 1.81 V and 1.90 V rather than the
# 0.45 V a datasheet would suggest - at 12 V that difference is 15% of the rail.
float32 back_emf_rpm_left
float32 back_emf_rpm_right

uint16 fault_flags
uint16 FAULT_LEFT_STALL=1
uint16 FAULT_RIGHT_STALL=2
uint16 FAULT_LEFT_OPEN=4
uint16 FAULT_RIGHT_OPEN=8
