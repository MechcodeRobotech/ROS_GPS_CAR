# How to reach the robot's web UI, latched so it is there the moment you look.
#
#   ros2 topic echo --once --field qr_ascii /gps_localize/web/connect
#
# Use --field. Plain `ros2 topic echo` renders a multi-line string with a blank
# line between every row, which doubles the QR's row spacing and stops it
# scanning - the code still LOOKS like a QR, which is the dangerous part.
std_msgs/Header header

# The address to hand a phone, and what qr_ascii encodes. This is an IP, not the
# .local name, on purpose: mDNS is dropped by guest Wi-Fi with client isolation,
# corporate multicast filtering, VLANs and most VPNs. An IP works wherever the
# two devices can reach each other at all. It goes stale when this machine
# changes address, which costs nothing here - read the topic again.
string url

# The friendlier form, better to type or remember, and it survives the machine
# changing address. Empty when there is no LAN address to publish.
string url_by_name

# Only useful on the machine running the stack.
string url_local

# The same address as a block-character QR, ready to point a camera at.
string qr_ascii
