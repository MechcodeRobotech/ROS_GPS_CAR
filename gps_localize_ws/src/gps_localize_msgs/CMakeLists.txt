cmake_minimum_required(VERSION 3.8)
project(gps_localize_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Telemetry.msg is deliberately dependency-free: every field is a float32, so
# micro-ROS can generate type support for it with nothing extra pulled into the
# firmware image. It is the ONLY message the ESP32 knows about, and the only one
# copied into firmware/extra_packages.
#
# The five *Status messages below are HOST-SIDE ONLY. They carry a Header, real
# integer types and named constants - all of which make them pleasant to consume
# and none of which the firmware should have to pay for. telemetry_split turns
# one compact frame from the board into these six typed topics, which keeps the
# air link to a single message per cycle instead of six.
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Telemetry.msg"
  "msg/RailStatus.msg"
  "msg/MotorStatus.msg"
  "msg/GpsStatus.msg"
  "msg/EstimatorStatus.msg"
  "msg/SafetyStatus.msg"
  "msg/WebConnect.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
