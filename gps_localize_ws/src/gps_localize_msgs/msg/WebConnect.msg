# How to reach the robot's web UI, latched so it is there the moment you look.
#
#   ros2 topic echo --once --field qr_ansi /gps_localize/web/connect
#
# Use --field. Plain `ros2 topic echo` prints the QR as one escaped, TRUNCATED
# line - and even untruncated it puts a blank line between every row, which
# doubles the QR's row spacing and stops it scanning. The code still LOOKS like
# a QR in both cases, which is the dangerous part.
#
# Use qr_ansi, not qr_ascii, for anything a camera will be pointed at. See the
# two fields at the bottom.
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

# The same address as a QR, in two forms. They differ only in how they survive
# the terminal they are printed into.
#
# qr_ansi states its own colours, so it is black-on-white whatever theme the
# terminal uses. THIS IS THE ONE TO POINT A CAMERA AT.
#
# qr_ascii is plain block characters, drawn in whatever colour the terminal
# uses for text. On a light theme that is a correct QR; on a dark theme it is an
# inverted one, which zbar rejects outright and many phone cameras will not
# read. It is here for the places colour cannot go - a log, a file, a page -
# and for pasting somewhere else.
#
# Not a guess: test_qr.py rasterises both the way a terminal paints them and
# reads them back with a real scanner, on both themes.
string qr_ansi
string qr_ascii
