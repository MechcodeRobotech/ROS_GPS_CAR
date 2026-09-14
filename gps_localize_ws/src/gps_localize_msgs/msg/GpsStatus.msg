# Receiver quality, alongside the position itself on /gps/fix.
#
# Separate from NavSatFix because these describe how much to TRUST that fix, and
# the estimator uses them to size its measurement variance: an RTK fix is given
# centimetre confidence, an unaugmented one metres.
std_msgs/Header header

uint8 fix_quality
uint8 FIX_NONE=0
uint8 FIX_GPS=1
uint8 FIX_DGPS=2
uint8 FIX_RTK_FIXED=4
uint8 FIX_RTK_FLOAT=5

uint8 satellites
float32 hdop

# Ground speed and course as the receiver reports them. Course is only
# meaningful while actually moving - a stationary receiver reports noise, which
# is why the estimator stops trusting it below a walking pace.
float32 ground_speed_mps
float32 course_deg
