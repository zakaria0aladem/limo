# Device setup — the one-time base every workflow needs

This is the shared foundation for all three workflows (regular AMCL nav, mocap
Nav2, and MATLAB control). Do it once. Then go to the workflow you want:
`NAVIGATION.md` (AMCL + SLAM), `OPTITRACK_NAV2_SETUP.md` (mocap), or
`CONTROL_SETUP.md` (control).

> Reconstructed from the project's connection/install notes (the original
> "setup your device" note wasn't in the repo). Verify the exact steps against
> your machine, especially the `limo_msgs` build and any AgileX image specifics.

## 1. The robot side (LiDAR + drivers) — vendor code, already on the LIMO

The LiDAR driver, wheel odometry, and base controller are **AgileX's stack**
(`limo_bringup`, `limo_base`, the YDLIDAR node), pre-installed on the LIMO. It is
not part of this repo — you start it, you don't build it:

```bash
ssh agilex@<LIMO_IP>
# clean any stale ROS processes
pkill -9 ros2
# the robot uses Fast DDS on domain 10, and must NOT source the UDP profile
unset FASTRTPS_DEFAULT_PROFILES_FILE
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
# bring up drivers + LiDAR
ros2 launch limo_bringup limo_start.launch.py
```

Wait for `connect the serial port: '/dev/ttyTHS0'` and `Now lidar is scanning...`.
Leave it running — closing it stops the robot. This publishes `/scan`, `/odom`,
`/tf`, and the LIMO's TF tree, which everything downstream consumes.

## 2. Network (both machines)

- Same subnet; IPs are DHCP, so verify each session (`hostname -I` on the LIMO).
- **`ROS_DOMAIN_ID=10`** and **`rmw_fastrtps_cpp`** on **both** the robot and the
  laptop container. A mismatch is the usual reason the container can't see robot
  topics.

## 3. The laptop container (once)

The laptop drives Nav2/RViz from a Foxy desktop container:

```bash
xhost +local:docker
sudo docker run -dit \
  --net=host --ipc=host --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/ros2_ws:/root/ros2_ws \
  -v ~/maps:/root/maps \
  -e ROS_DOMAIN_ID=10 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --name limo_laptop \
  osrf/ros:foxy-desktop
```

`~/ros2_ws` and `~/maps` on the laptop are mounted into the container at
`/root/ros2_ws` and `/root/maps` — that pairing is how files cross the boundary.
Re-enter later with `sudo docker start limo_laptop && sudo docker exec -it limo_laptop bash`.

## 4. Build `limo_msgs` in the container (once)

Without it you get `Deserialization of data failed` on `/limo_status`.

```bash
# put this repo's src/limo_msgs into the mounted workspace, then:
cd /root/ros2_ws
colcon build --packages-select limo_msgs
source install/setup.bash    # add to ~/.bashrc so every shell has it
```

## 5. Install Nav2 (once)

```bash
apt update && apt install -y ros-foxy-navigation2 ros-foxy-nav2-bringup
```

## 6. Map + config files into `~/maps` (once)

The launch files read from `/root/maps` (= laptop `~/maps`):

```bash
cp config/{mapMTR5.yaml,mapMTR5.pgm,nav2.yaml,slam_params.yaml,fastdds_udp.xml} ~/maps/
```

## 7. Mocap-only extras

If you'll use OptiTrack, also install `ros-foxy-vrpn-mocap` and `netbase`, and
build `mocap_localization`. Full steps: `OPTITRACK_NAV2_SETUP.md`.

---

## Where to go next

| You want to… | Go to |
|---|---|
| Build a map / navigate with the onboard LiDAR (AMCL, SLAM) | `NAVIGATION.md` |
| Navigate with OptiTrack absolute localization | `OPTITRACK_NAV2_SETUP.md` → `OPTITRACK_NAV2_DAILY.md` |
| Run closed-loop P/PID/LQR control from MATLAB | `CONTROL_SETUP.md` → `CONTROL_DAILY.md` |
| Just confirm the robot moves (no map) | `WANDERING.md` |
