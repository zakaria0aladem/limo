# ============================================================================
# Dockerfile - Foxy container preloaded for the LIMO mocap/Nav2 stack.
# ----------------------------------------------------------------------------
# TODO(zakaria): reconstructed stub. The original baked-image steps were not
# recovered. This mirrors the manual setup in docs/SETUP.md; verify, then
# remove this banner. Build:  docker build -t limo_foxy:mocap-ready .
# ============================================================================
FROM osrf/ros:foxy-desktop

# netbase: provides /etc/protocols so VRPN can open its UDP socket
# (fixes "getprotobyname() failed / VRPN connection is bad")
RUN apt-get update && apt-get install -y \
        ros-foxy-vrpn-mocap \
        ros-foxy-navigation2 \
        ros-foxy-nav2-bringup \
        netbase \
    && rm -rf /var/lib/apt/lists/*

# Fast DDS UDP-only profile (container<->host SHM fix). Mount ~/maps at runtime
# so this path resolves, or COPY the file in and adjust the path.
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/maps/fastdds_udp.xml
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_DOMAIN_ID=10

CMD ["bash"]
