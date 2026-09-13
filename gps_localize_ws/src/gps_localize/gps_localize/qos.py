"""QoS profiles for the topics the board feeds.

WHY THE STREAMING TOPICS ARE BEST-EFFORT

Telemetry, IMU, odometry and the GPS fix are *streams*. Each message replaces
the one before it, and a frame that arrives late is worth less than no frame at
all - nothing downstream reconstructs history from them, and every consumer
already checks staleness rather than assuming continuity.

Delivering them reliably is not merely unnecessary, it is actively harmful on
this robot. micro-ROS publishes over micro-XRCE-DDS, and a RELIABLE publisher
blocks inside `rcl_publish` until the agent acknowledges. That call sits in the
firmware's 100 Hz control callback, so every retransmission on a slow or lossy
Wi-Fi path stretches the control loop directly. Measured on hardware: with every
publisher reliable, the control loop ran at **7.5 Hz instead of 100 Hz**, and
the link carried about ten packets for each message actually published - the
rest was retransmission traffic. Control, safety timing and the EKF prediction
step all run in that loop, so this is a safety property, not a throughput one.

Commands, parameters, configuration and the log stay RELIABLE. Those are events,
not streams: losing "stop" or "here is your new parameter" is not recoverable by
waiting for the next one.

THE MIGRATION ORDER MATTERS, AND ONLY ONE ORDER IS SAFE

DDS reliability compatibility is asymmetric:

    publisher RELIABLE    + subscriber BEST_EFFORT  -> match
    publisher BEST_EFFORT + subscriber RELIABLE     -> NO MATCH, silently

So the host subscribers move to best-effort FIRST, while the firmware is still
publishing reliably - that combination matches. The firmware publishers move
second, whenever the board can next be flashed. At no point is there a window
where the two sides fail to match, which matters because a QoS mismatch in ROS 2
produces no error at all: the topic still appears in `ros2 topic list`, the
publisher still counts as connected, and not one message is delivered. This
project has been caught by exactly that twice.
"""

from __future__ import annotations

from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

# The topics the firmware streams. Keep this list and the firmware's
# best-effort publishers in step - tools/qc.py checks that they agree.
STREAM_TOPICS = ('/telemetry', '/imu/data', '/odom', '/gps/fix', '/params/values')


# /params/values joined this list for the same reason as the sensor streams,
# and it arrived by measurement. It is published every two seconds from inside
# the control callback, and a RELIABLE publish on micro-ROS BLOCKS there until
# the agent acknowledges - which is what put a 129 ms spike into a loop whose
# period is 10 ms. It got worse as the parameter table grew from 24 entries to
# 85. Nothing is lost by dropping a sample: the next one is two seconds away.


def stream_qos(depth=5):
    """For sensor streams: the newest frame matters, a lost one does not."""
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def latched_qos(depth=1):
    """For values a late subscriber must still receive - versions, config."""
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )
