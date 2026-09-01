/**
 * @file robot.h
 * @brief Physical robot: wheels, track width, top speed.
 *
 * MEASURE THESE ON YOUR ROBOT. Getting the track width wrong makes the
 * reported wheel speeds and the manual turn rate wrong; getting the wheel
 * diameter wrong scales every speed.
 *
 * The same numbers live in the ROS side `robot.yaml` and in
 * `pid.yaml: estimator.track_m` - keep all three in step.
 */
#ifndef ROBOT_H
#define ROBOT_H

#define WHEEL_DIAMETER_M        0.081f    // 81 mm wheel
#define WHEEL_TRACK_M           0.300f    // >>> MEASURE <<< centre of left wheel to centre of right

// JGA25-370-100RPM gearmotor on 3S LiPo (11.1 V nominal, 12.6 V full)
#define MOTOR_MAX_RPM           100.0f    // output shaft rpm at 12 V (datasheet)
#define MOTOR_RPM_RATIO         0.85f     // usable fraction: battery sag + safety margin

// Top ground speed = (rpm/60) * pi * D  ->  100/60 * pi * 0.081 = 0.424 m/s
#define ROBOT_MAX_SPEED_MPS     (MOTOR_MAX_RPM * MOTOR_RPM_RATIO / 60.0f * 3.14159265f * WHEEL_DIAMETER_M)
#define ROBOT_DEFAULT_SPEED_MPS 0.25f     // used when nothing else sets a speed

//--------------------------- where things sit ----------------------------//
// This robot has two driven wheels and two casters, so it turns about the
// midpoint of the DRIVE AXLE - not about the middle of the chassis, and not
// about wherever a sensor happens to be bolted.
//
// That matters because a sensor mounted away from the turning centre travels
// through an arc when the robot rotates on the spot. A GPS antenna 30 cm ahead
// of the axle sweeps a 30 cm radius circle while the robot's position has not
// changed at all, and the estimator would read that as real movement. Removing
// it is a rotation and a subtraction - the "lever arm" correction - and it
// needs to know where the antenna actually is.
//
// FRAME: x forward, y left, metres, measured FROM THE DRIVE AXLE MIDPOINT.
// So a sensor mounted ahead of the axle has positive x; one to the right has
// negative y. Measure with a tape from the midpoint between the two drive
// wheels to the sensor, not from a corner of the chassis.
//
//        front
//          ^ +x
//          |
//   +y <---o          o = drive axle midpoint, the turning centre
//          |
//        casters
//
// These default to zero, which is the same as not correcting at all - so a
// robot with unmeasured offsets behaves exactly as it did before.
#define GPS_OFFSET_X_M          0.00f     // >>> MEASURE <<< antenna ahead of the axle (+) or behind (-)
#define GPS_OFFSET_Y_M          0.00f     // >>> MEASURE <<< antenna left of centre (+) or right (-)
#define IMU_OFFSET_X_M          0.00f     // >>> MEASURE <<< IMU ahead of the axle (+) or behind (-)
#define IMU_OFFSET_Y_M          0.00f     // >>> MEASURE <<< IMU left of centre (+) or right (-)

// Where the drive axle sits relative to the middle of the chassis, forward
// positive. Recorded so the body frame and any future footprint agree with the
// physical machine; it does not affect the lever-arm maths above, which is all
// measured from the axle already.
#define DRIVE_AXLE_OFFSET_X_M   0.00f     // >>> MEASURE <<< axle ahead of chassis centre (+)

#endif // ROBOT_H
