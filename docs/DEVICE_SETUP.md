# Device setup — the one-time base every workflow needs

This is the shared foundation for all three workflows (regular AMCL nav, mocap
Nav2, and MATLAB control). Do it once. Then go to the workflow you want:
[`NAVIGATION.md`](NAVIGATION.md) (AMCL + SLAM),
[`OPTITRACK_NAV2_SETUP.md`](OPTITRACK_NAV2_SETUP.md) (mocap), or
[`CONTROL_SETUP.md`](CONTROL_SETUP.md) (control).

> Verify these steps against your machine, especially the `limo_msgs` build.

## 1. The robot side (LiDAR + drivers) — vendor code, already on the LIMO

The LiDAR driver, wheel odometry, and base controller are **AgileX's stack**
(`limo_bringup`, `limo_base`, the YDLIDAR node), pre-installed on the LIMO. It is
not part of this repo — you start it, you don't build it:

```bash
ssh agilex@<LIMO_IP>
```

```bash
# clean any stale ROS processes
pkill -9 ros2
```

```bash
# the robot uses Fast DDS on domain 10, and must NOT source the UDP profile
unset FASTRTPS_DEFAULT_PROFILES_FILE
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
```

```bash
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
Re-enter later with:

```bash
sudo docker start limo_laptop && sudo docker exec -it limo_laptop bash
```

> [!warning] A rebuilt container loses everything you apt-installed.
> `docker run` on a fresh image starts blank — `netbase`, `vrpn_mocap`, and any
> apt packages from a previous container are gone. Re-run steps 4–7 after any
> rebuild. (The repo `Dockerfile` bakes these in to avoid the re-do — prefer
> building from it once it's verified.)

## 4. Install system dependencies in the container (once)

```bash
apt update
apt install -y netbase ros-foxy-navigation2 ros-foxy-nav2-bringup
```

- **`netbase` is required** — without it `vrpn_mocap` fails with
  `getprotobyname() failed` / "connection is bad". Easy to forget; a fresh
  container does **not** have it.
- Verify: `dpkg -l | grep netbase` (should list it) and
  `ros2 pkg list | grep navigation2`.

## 5. Build `limo_msgs` in the container (once)

Without it you get `Deserialization of data failed` on `/limo_status` (battery,
mode, error code). Copy this repo's `src/limo_msgs` into `~/ros2_ws/src`, then:

```bash
cd /root/ros2_ws
colcon build --packages-select limo_msgs
source install/setup.bash    # add to ~/.bashrc so every shell has it
```

> [!warning] "Duplicate package names not supported: limo_msgs"
> The AgileX package set (`src/limo_ros2/limo_msgs`) already contains an
> identical `limo_msgs`. If you also drop this repo's copy in `src/limo_msgs`,
> colcon sees two and refuses. They define the same message, so **keep one**:
> ```bash
> rm -rf /root/ros2_ws/src/limo_msgs      # keep AgileX's src/limo_ros2/limo_msgs
> colcon build --packages-select limo_msgs
> ```
> `LimoStatus.msg` fields (confirmed): `header, vehicle_state, control_mode,`
> `battery_voltage, error_code, motion_mode`.

> [!note] Build only what you name.
> Always use `--packages-select`. Plain `colcon build` from a home dir crawls
> every `setup.py` it can find (MATLAB engine, venvs, numpy tests) and throws a
> wall of unrelated errors. Run it from `/root/ros2_ws`, never from `~`.

## 6. Map + config files into `~/maps` (once)

The launch files read from `/root/maps` (= laptop `~/maps`). Copy from the repo's
`config/` **while inside the cloned repo folder**:

```bash
cd <path-to-cloned-repo>          # NOT ~/ros2_ws — config/ lives in the repo
cp config/{mapMTR5.yaml,mapMTR5.pgm,nav2.yaml,fastdds_udp.xml,limo_mocap_nav2.launch.py} ~/maps/
```

> [!note] If `~/maps` already has these, skip this step.
> `~/maps` is a host mount and survives container rebuilds. Check with
> `ls ~/maps/` first — if the files are there, you're done. The `cannot stat
> 'config/...'` error means you ran the copy from the wrong folder (there's no
> `config/` in `~/ros2_ws`); `cd` into the repo first.

## 7. Mocap extras (only for the OptiTrack workflow)

```bash
apt install -y ros-foxy-vrpn-mocap
# then build the localizer:
cp -r <repo>/src/mocap_localization /root/ros2_ws/src/
cd /root/ros2_ws && colcon build --packages-select mocap_localization
source install/setup.bash
```

Verify: `ros2 pkg list | grep vrpn_mocap`. Full steps:
[`OPTITRACK_NAV2_SETUP.md`](OPTITRACK_NAV2_SETUP.md).

## 8. DDS profile for MATLAB / cross-boundary (only if using MATLAB)

MATLAB (or any host process) can't see the container's topics over shared memory
(root-owned segments). Force UDP:

```bash
# in every container shell that MATLAB must talk to:
export FASTRTPS_DEFAULT_PROFILES_FILE=/root/maps/fastdds_udp.xml
```

Details and the MATLAB side: [`CONTROL_SETUP.md`](CONTROL_SETUP.md).

---

## Where to go next

| You want to… | Go to |
|---|---|
| Build a map / navigate with the onboard LiDAR (AMCL, SLAM) | [`NAVIGATION.md`](NAVIGATION.md) |
| Navigate with OptiTrack absolute localization | [`OPTITRACK_NAV2_SETUP.md`](OPTITRACK_NAV2_SETUP.md) → [`OPTITRACK_NAV2_DAILY.md`](OPTITRACK_NAV2_DAILY.md) |
| Run closed-loop P/PID/LQR control from MATLAB | [`CONTROL_SETUP.md`](CONTROL_SETUP.md) → [`CONTROL_DAILY.md`](CONTROL_DAILY.md) |
| Just confirm the robot moves (no map) | [`WANDERING.md`](WANDERING.md) |

---
_Part of the [LIMO documentation index](../README.md#documentation) · [repo home](../README.md)._
