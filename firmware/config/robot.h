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

#endif // ROBOT_H
