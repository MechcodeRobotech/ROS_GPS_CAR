# The fused pose and heading, and - just as important - where each came from.
#
# Every sensor here can fail without stopping the robot, so the SOURCE fields
# are not diagnostics. They are how you tell a heading held by a compass from
# one being carried by gyro alone, which look identical until the robot drifts.
std_msgs/Header header

float32 heading_deg              # fused, compass degrees, 0 = north, clockwise
float32 target_heading_deg
float32 heading_error_deg

float32 speed_mps                # fused body speed
float32 target_speed_mps
float32 yaw_rate_dps             # counter-clockwise positive

# Local metric position, metres from the datum set at the first valid fix.
float32 x
float32 y

# How much the filter trusts that position, and whether GPS is currently holding
# it down. While coasting through a dropout the sigma grows honestly rather than
# the estimate simply looking confident.
float32 position_sigma_m
bool position_anchored

# Ranked best first. The estimator falls down this ladder as sensors drop out.
uint8 heading_source
uint8 HEADING_GPS_COURSE=0
uint8 HEADING_COMPASS=1
uint8 HEADING_IMU_MAG=2
uint8 HEADING_GYRO_ONLY=3

uint8 speed_source
uint8 SPEED_FUSED=0
uint8 SPEED_GPS=1
uint8 SPEED_MOTOR_MODEL=2

uint8 sensor_health              # bitfield: which sensors are answering
uint8 HEALTH_GPS=1
uint8 HEALTH_IMU=2
uint8 HEALTH_COMPASS=4

float32 mag_heading_deg          # the compass on its own, before fusion
uint8 mag_status
uint8 mag_calib_state
