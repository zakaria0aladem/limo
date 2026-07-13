# Device setup — the one-time base every workflow needs

This is the shared foundation for all three workflows (regular AMCL nav, mocap
Nav2, and MATLAB control). Do it once. Then go to the workflow you want:
[`NAVIGATION.md`](NAVIGATION.md) (AMCL + SLAM),
[`OPTITRACK_NAV2_SETUP.md`](OPTITRACK_NAV2_SETUP.md) (mocap), or
[`CONTROL_SETUP.md`](CONTROL_SETUP.md) (control).

Verify these steps against your machine, especially the `limo_msgs` build.

## 0. Get the repo (once)

Clone it on the **laptop host**. These docs assume it lives at `~/limo`:

```bash
cd ~
git clone https://github.com/zakaria0aladem/limo.git
```

Three different locations are involved — keep them straight:

- `~/limo` — the cloned repo (source of the packages and config files)
- `~/ros2_ws` — the ROS 2 workspace you build in (mounted into the container)
- `~/maps` — runtime config the launch files read (mounted into the container)

Later steps copy *from* `~/limo` *into* `~/ros2_ws/src` and `~/maps`. If you cloned
elsewhere, substitute your path for `~/limo` everywhere below.

## 1. The robot side (LiDAR + drivers) — vendor code, already on the LIMO

The LiDAR driver, wheel odometry, and base controller are **AgileX's stack**
(`limo_bringup`, `limo_base`, the YDLIDAR node), pre-installed on the LIMO. It is
not part of this repo — you start it, you don't build it:

```bash
ssh agilex@192.168.8.185       # example IP on the GL router; verify with hostname -I
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
  --name limo_laptop \  #you can change the image name here
  osrf/ros:foxy-desktop
```

`~/ros2_ws` and `~/maps` on the laptop are mounted into the container at
`/root/ros2_ws` and `/root/maps` — that pairing is how files cross the boundary.

Re-enter later with:

```bash
sudo docker start limo_laptop && sudo docker exec -it limo_laptop bash
```

You are inside the container when the prompt reads `root@...:/#`. Every `apt` and
`colcon` step below runs **inside** the container, not on the host.

**A rebuilt container loses everything you apt-installed.** `docker run` on a fresh
image starts blank — `netbase`, `vrpn_mocap`, and any apt packages from a previous
container are gone. Re-run steps 4–7 after any rebuild. (The repo `Dockerfile` bakes
these in to avoid the re-do — prefer building from it once it's verified.)

## 4. Install system dependencies in the container (once)

```bash
apt update
apt install -y netbase ros-foxy-navigation2 ros-foxy-nav2-bringup ros-foxy-vrpn-mocap
```

- **`netbase` is required** — without it `vrpn_mocap` fails with
  `getprotobyname() failed` / "connection is bad". A fresh container does **not**
  have it.
- `vrpn_mocap` is only needed for the OptiTrack workflow, but it's installed here so
  everything is in one place.
- Verify: `dpkg -l | grep netbase` and
  `ros2 pkg list | grep -E "navigation2|vrpn_mocap"`.

## 5. Build `limo_msgs` in the container (once)

Without it you get `Deserialization of data failed` on `/limo_status` (battery,
mode, error code).

`limo_msgs` is very likely **already in `~/ros2_ws/src`** — AgileX ships it inside
`limo_ros2/`. Check first:

```bash
ls ~/ros2_ws/src/limo_ros2/limo_msgs      # AgileX's copy
```

If it's there, do **not** add another copy (that causes the duplicate error below).
Just build it:

```bash
cd /root/ros2_ws
colcon build --packages-select limo_msgs
source install/setup.bash
```
To source it automatically in every new shell, add these to ~/.bashrc. A fresh
container does not source ROS at all — you need the base install first, then the
workspace overlay on top, or you get ros2: command not found:

```bash
echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc          # base ROS 2 -- gives the `ros2` command
echo 'source /root/ros2_ws/install/setup.bash' >> ~/.bashrc  # your workspace overlay (limo_msgs, etc.)
echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/root/maps/fastdds_udp.xml' >> ~/.bashrc
```

These apply to new shells. For the current shell, run the two source lines by
hand once:

```bash
source /opt/ros/foxy/setup.bash
source /root/ros2_ws/install/setup.bash
which ros2        # should print /opt/ros/foxy/bin/ros2
```

Order matters: the workspace overlay layers on top of the base install and does not
provide ros2 by itself. Sourcing only install/setup.bash (without the base first)
still leaves ros2: command not found.


**"Duplicate package names not supported: limo_msgs"** — the AgileX set
(`src/limo_ros2/limo_msgs`) already contains an identical `limo_msgs`. If a second
copy also sits in `src/limo_msgs`, colcon refuses. They define the same message, so
keep one:

```bash
rm -rf /root/ros2_ws/src/limo_msgs      # keep AgileX's src/limo_ros2/limo_msgs
colcon build --packages-select limo_msgs
```

`LimoStatus.msg` fields (confirmed): `header, vehicle_state, control_mode,
battery_voltage, error_code, motion_mode`.

**Build only what you name.** Always use `--packages-select`. Plain `colcon build`
from a home dir crawls every `setup.py` it can find (MATLAB engine, venvs, numpy
tests) and throws a wall of unrelated errors. Run it from `/root/ros2_ws`, never
from `~`.

## 6. Map + config files into `~/maps` (once)

The launch files read from `/root/maps` (= laptop `~/maps`). Copy from the repo's
`config/`. Run this on the **host** (where `~/limo` and `~/maps` both live):

```bash
cp ~/limo/config/{mapMTR5.yaml,mapMTR5.pgm,nav2.yaml,fastdds_udp.xml,limo_mocap_nav2.launch.py} ~/maps/
```

**If `~/maps` already has these, skip this step.** `~/maps` is a host mount and
survives container rebuilds. Check with `ls ~/maps/` first. A `cannot stat
'config/...'` error means you ran the copy from the wrong folder — use the full
`~/limo/config/...` path as above.

## 7. Build the mocap localizer (only for the OptiTrack workflow)

`vrpn_mocap` was already installed in step 4. Build the localizer package:

```bash
# it's likely already in the workspace; check:
ls ~/ros2_ws/src/mocap_localization
# if missing, copy it in on the host:  cp -r ~/limo/src/mocap_localization ~/ros2_ws/src/
cd /root/ros2_ws && colcon build --packages-select mocap_localization
source install/setup.bash
```

Verify: `ros2 pkg list | grep -E "vrpn_mocap|mocap_localization"`. Full steps:
[`OPTITRACK_NAV2_SETUP.md`](OPTITRACK_NAV2_SETUP.md).

## 8. DDS profile for MATLAB / cross-boundary (only if using MATLAB)

MATLAB (or any host process) can't see the container's topics over shared memory
(root-owned segments). Force UDP — set the profile on **both** sides:

```bash
# in every CONTAINER shell that MATLAB must talk to:
export FASTRTPS_DEFAULT_PROFILES_FILE=/root/maps/fastdds_udp.xml

# in MATLAB, BEFORE any ros2 object (host path -- adjust user):
#   setenv("FASTRTPS_DEFAULT_PROFILES_FILE","/home/zakaria/maps/fastdds_udp.xml")
```

The container path (`/root/maps/...`) and the host/MATLAB path
(`/home/<user>/maps/...`) point at the **same file** through the mount. Details and
the MATLAB side: [`CONTROL_SETUP.md`](CONTROL_SETUP.md).

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
