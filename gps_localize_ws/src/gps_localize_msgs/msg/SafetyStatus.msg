# Why the robot is or is not allowed to move, and how the board itself is doing.
#
# stop_flags is the whole reason this is one message: any single flag stops the
# robot, and knowing WHICH one is the difference between a flat battery, a
# pressed mushroom and a dropped network.
std_msgs/Header header

uint8 run_state
uint8 STATE_IDLE=0
uint8 STATE_AUTO=1
uint8 STATE_MANUAL=2
uint8 STATE_ESTOP=3
uint8 STATE_COMM_LOSS=4

uint32 stop_flags
uint32 STOP_HW_EMERGENCY=1       # IO13, the mushroom switch
uint32 STOP_ESP_BUTTON=2         # a second on-board button, not fitted here
uint32 STOP_SW_ESTOP=4           # from the web UI
uint32 STOP_HEARTBEAT_LOST=8     # the browser stopped talking
uint32 STOP_CMD_TIMEOUT=16
uint32 STOP_AGENT_LOST=32        # micro-ROS agent unreachable
uint32 STOP_BATTERY_LOW=64       # LATCHED - cleared only by a power cycle
uint32 STOP_STARTUP=128          # boot lockout, clears itself
uint32 STOP_BATTERY_SOFT=256     # stop driving, stay powered

# The main contactor. The pack sits behind it, so nothing can measure the
# battery while this is false - which is exactly why the battery cutoff latches
# until a power cycle rather than un-latching when the reading disappears.
bool relay_closed

# Watch free_heap on the web UI: it must stay flat. A slow leak on a board that
# runs for hours shows up here long before it shows up as a crash.
uint32 free_heap_bytes
uint32 loop_us

# Why the board last restarted, and how long it has been up. Together these are
# how a brownout is caught: a 3.3 V rail that sags under Wi-Fi bursts produces
# ESP_RST_BROWNOUT and an uptime that keeps starting over, and nothing else in
# the system would show it.
uint8 reset_reason
uint8 RESET_POWERON=1
uint8 RESET_SOFTWARE=3
uint8 RESET_PANIC=4
uint8 RESET_INT_WATCHDOG=5
uint8 RESET_TASK_WATCHDOG=6
uint8 RESET_WATCHDOG=7
uint8 RESET_BROWNOUT=9
float32 uptime_s
