# GPS_Localize - ROS 2 Humble workspace and web UI, no ROS install on the host.
FROM ros:humble-ros-base

# Base tools. curl is needed by the compose healthcheck, python3-pip by the
# PlatformIO layer below, and git by the agent build. The lists are kept for
# the next layer and cleared at the end of it.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git curl python3-pip \
    && rm -rf /var/lib/apt/lists/*

# micro-ROS agent. There is no apt package for Humble, and building
# Micro-XRCE-DDS-Agent directly does not work: its superbuild pins a FastDDS
# tag (2.12.x) that no longer exists upstream, and with the superbuild off it
# needs microxrcedds_client, which the base image does not ship.
#
# micro_ros_setup is the supported route and resolves those versions itself.
# It is also exactly how the agent on the development machine was built, so
# the container and the host end up with the same thing.
# apt lists were cleared by the layer above, so rosdep cannot resolve anything
# without refreshing them first - it fails on libcurl4-openssl-dev otherwise.
# They are cleared again at the end of this layer.
RUN apt-get update \
    && mkdir -p /uros_ws/src \
    && git clone -b humble --depth 1 \
        https://github.com/micro-ROS/micro_ros_setup.git /uros_ws/src/micro_ros_setup \
    && . /opt/ros/humble/setup.sh \
    && cd /uros_ws \
    && rosdep install --from-paths src --ignore-src -y \
    && colcon build \
    && . install/local_setup.sh \
    && ros2 run micro_ros_setup create_agent_ws.sh \
    && ros2 run micro_ros_setup build_agent.sh \
    && rm -rf /uros_ws/build /uros_ws/log /var/lib/apt/lists/*

# PlatformIO, so the ESP32 firmware can be built and flashed from the same
# container. Kept in its own layer: it is large and changes rarely.
RUN pip3 install --no-cache-dir platformio

WORKDIR /ws
COPY gps_localize_ws/src ./src
COPY firmware /firmware
# micro_ros_agent is built from source in /uros_ws, so rosdep can never
# resolve it as a system package - skip that key rather than let it fail the
# whole install. The agent workspace is sourced so colcon can still find it.
RUN . /opt/ros/humble/setup.sh \
    && . /uros_ws/install/local_setup.sh \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src -y \
        --skip-keys micro_ros_agent \
    && colcon build --symlink-install \
    && rm -rf /var/lib/apt/lists/*

COPY docker-entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/docker-entrypoint.sh \
    # docker exec does NOT run the entrypoint, so an interactive shell would
    # have no ros2 on PATH - 'docker compose exec gps_localize ros2 topic list'
    # failed with "executable file not found" even though ROS was installed.
    # Sourcing from .bashrc fixes the interactive case without affecting how
    # the service itself starts.
    && printf '%s\n' \
        'source /opt/ros/humble/setup.bash' \
        'source /uros_ws/install/local_setup.bash' \
        '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' \
        'export ROS_DOMAIN_ID=10' \
        >> /root/.bashrc

# Flashing needs the board's serial port; docker-compose passes /dev/ttyUSB0
# through. Build the firmware with:
#   docker compose exec gps_localize pio run -e wifi -d /firmware
# and flash by adding -t upload.
# Fast DDS prefers shared memory over UDP once it has discovered a peer on the
# same host, and shared-memory segments do not cross the container boundary.
# The effect is worse than a plain failure: discovery succeeds, so `ros2 topic
# list` on the host shows every container topic with the right types and the
# right subscriber counts, and then not one sample ever arrives. Restricting
# the container to UDPv4 costs nothing at 10 Hz and makes the host's ros2 CLI
# work against the container with no setup on the host side.
RUN printf '%s\n' \
    '<?xml version="1.0" encoding="UTF-8" ?>' \
    '<dds xmlns="http://www.eprosima.com">' \
    '  <profiles>' \
    '    <transport_descriptors>' \
    '      <transport_descriptor>' \
    '        <transport_id>udp_only</transport_id>' \
    '        <type>UDPv4</type>' \
    '      </transport_descriptor>' \
    '    </transport_descriptors>' \
    '    <participant profile_name="udp_participant" is_default_profile="true">' \
    '      <rtps>' \
    '        <userTransports><transport_id>udp_only</transport_id></userTransports>' \
    '        <useBuiltinTransports>false</useBuiltinTransports>' \
    '      </rtps>' \
    '    </participant>' \
    '  </profiles>' \
    '</dds>' \
    > /etc/fastdds_udp_only.xml

ENV FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_udp_only.xml
ENV ROS_DOMAIN_ID=10
ENV PLATFORMIO_CORE_DIR=/root/.platformio
EXPOSE 8080 8888/udp
ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["ros2", "launch", "gps_localize", "bringup.launch.py"]
