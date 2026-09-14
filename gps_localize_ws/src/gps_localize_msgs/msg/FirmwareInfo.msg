# What firmware the board is running, and whether the host is ahead of it.
#
# Latched, so it is there the moment you ask rather than on the next publish.
#
# The board and the host update separately - the host pulls from git, the board
# needs a flash - so they drift apart routinely. An older board must keep
# working: it still connects, drives and reports. What it must not do is drift
# SILENTLY, because "a telemetry field that is always zero" and "firmware that
# predates that field" look identical from outside.
#
# The version arrives inside the ordinary telemetry frame as a single number,
# not on its own topic from the board. A string or a Header would drag std_msgs
# into the firmware image, and Telemetry.msg is deliberately the only message
# micro-ROS has to compile type support for. This message is assembled on the
# host, where none of that costs anything.
std_msgs/Header header

uint16 version           # major * 100 + minor, as the board reports it
uint16 version_major
uint16 version_minor

uint16 host_expects      # the version this host was built alongside
bool board_is_older      # a notice, never a block - an old board still drives
string summary           # one line fit to show a person
