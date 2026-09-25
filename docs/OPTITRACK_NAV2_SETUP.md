# Mocap absolute positioning — first-time setup

**What this adds:** it replaces Nav2's AMCL (LiDAR guess + manual **2D Pose
Estimate**) with **OptiTrack absolute positioning**. Afterwards the robot knows
its true position automatically: no pose initialization, no drift. A custom node
publishes `map → odom` from the mocap pose; the rest of Nav2 is unchanged.

**One-time only.** Do this once. The per-session run is in
[`OPTITRACK_NAV2_DAILY.md`](OPTITRACK_NAV2_DAILY.md). Finish by saving a container
image (Part 6) so you never repeat it. The background, design decisions and full
bug log are in [`OPTITRACK_NAV2_PROJECT.md`](OPTITRACK_NAV2_PROJECT.md).

---

## Prerequisites (from DEVICE_SETUP)

These come from [`DEVICE_SETUP.md`](DEVICE_SETUP.md). Don't redo them:

- Foxy container `limo_laptop` (`osrf/ros:foxy-desktop`), `--net=host`, domain
  **10**, **`rmw_fastrtps_cpp`**, mounts `~/ros2_ws` + `~/maps`.
- `limo_msgs` built in the container (otherwise "Deserialization of data failed").
- Nav2 installed: `ros-foxy-navigation2 ros-foxy-nav2-bringup`.
- Map files in `/root/maps/`: `mapMTR5.yaml`, `mapMTR5.pgm`, `nav2.yaml`.

**Use Fast DDS, not Cyclone.** Everything in this project uses
**`rmw_fastrtps_cpp`** on **domain 10**, on both the robot and the container. (Older
notes contain one stray `rmw_cyclonedds_cpp` line; ignore it.) A mismatch makes
Nav2 flaky or invisible.

---

## Part 1 — OptiTrack / Motive (Windows PC)

The Motive PC and the LIMO must share a subnet.

**IPs are DHCP; verify them each session.** Last seen: **Motive PC
`192.168.8.184`** (wired), **LIMO `192.168.8.185`** (Wi-Fi). Confirm with
`ipconfig` (PC) and `hostname -I` (LIMO) before relying on them.

**Steps in Motive:**

1. Confirm the capture volume is **calibrated** (status panel). If not, run wand
   calibration + ground plane.
2. Stick **≥4 markers** on the LIMO in an **asymmetric, varied-height** pattern
   (avoid squares and lines, which cause flips).
3. Select the markers → right-click → **Rigid Body → Create From Selected**.
   Rename it **`Limo`** (capital L, **no spaces**: VRPN drops names with spaces).
   Streaming ID `5`.
4. Fix the orientation: **Builder pane → Rigid Bodies → Edit**. Square the robot to
   a world axis, then **Reset Orientation**. The robot's front should point along
   Motive's +X when yaw reads 0.
5. Rigid-body tuning (Properties): **Forward Prediction → 0**, **Smoothing 0**,
   **Min Marker Count 3**, **Max Deflection ~5–20 mm**.
6. **Data Streaming pane:** enable streaming · **Local Interface = `192.168.8.184`**
   (the PC's IP on the robot's subnet, not loopback) · **Up Axis = Z Up** · Stream
   Rigid Bodies On · **VRPN On, port `3883`**.

Why these settings (learned the hard way):

- **Forward Prediction 0:** extrapolation fights a slow robot; 200 caused flips on
  sudden stops.
- **Up Axis Z:** ROS is Z-up (REP-103); Motive defaults to Y-up. With Y-up, the
  robot's "yaw" comes out as pitch and x/y are swapped.
- **Min Marker Count 3:** survives one occluded marker.
- Residual flips/drops in some arena corners are camera coverage gaps. Accepted,
  and handled downstream by `jump_threshold`.

---

## Part 2 — Install the VRPN driver (container)

On the laptop host:

```bash
sudo docker exec -it limo_laptop bash
```

Inside the container:

```bash
apt update && apt install -y ros-foxy-vrpn-mocap netbase
```

**`getprotobyname() failed` / "VRPN connection is bad":** the minimal container
lacks `/etc/protocols`, so VRPN can't open its UDP socket. **`netbase`** provides
that file. Always install `netbase` on a fresh container. (A `vrpn ver 07.35 vs
07.33` mismatch warning is harmless.)

**Test:**

```bash
ros2 launch vrpn_mocap client.launch.yaml server:=192.168.8.184 port:=3883
# second container shell:
ros2 topic echo /vrpn_mocap/Limo/pose      # numbers move when the robot moves
ros2 topic hz   /vrpn_mocap/Limo/pose      # ~100 Hz
```

The topic is **`/vrpn_mocap/Limo/pose`** (`geometry_msgs/PoseStamped`, frame
`world`). The name comes from the Motive rigid body name, so a different name
means a different topic.

---

## Part 3 — Install the `mocap_localization` package

This package's `mocap_map_odom` node publishes `map → odom` from the mocap pose,
replacing AMCL.

On the **laptop host**, copy the package into the mounted workspace:

```bash
cp -r ~/limo/src/mocap_localization ~/ros2_ws/src/
```

In the **container**, build it (or run the bundled script, which also installs
`vrpn_mocap` + `netbase` and builds):

```bash
cd /root/ros2_ws
colcon build --packages-select mocap_localization --symlink-install
source install/setup.bash
# alternative: bash /root/ros2_ws/src/mocap_localization/install_mocap_localization.sh
```

Verify: `ros2 pkg list | grep mocap_localization`.

**The math it implements:**

```
map→odom = (map→base) ∘ (odom→base)⁻¹
```

`map→base` = the mocap pose (2D: x, y, yaw); `odom→base` = the robot's wheel
odometry (left untouched). The node absorbs all drift into `map→odom`, which is
exactly AMCL's contract.

**Tunable parameters.** Pass them as launch arguments; no file edits needed:

```bash
ros2 launch mocap_localization mocap_localization.launch.py reg_yaw:=0.02 jump_threshold:=0.3
```

| Param | Default | Use |
|---|---|---|
| `reg_x`, `reg_y`, `reg_yaw` | 0 | Correct map↔world misalignment **without** rebuilding the map (m, m, rad) |
| `jump_threshold` | 0 (off) | e.g. `0.3` → reject mocap flips/dropouts larger than 0.3 m |
| `jump_accept_frames` | 10 | (node param) after a real jump, accept the new position once it holds for this many frames |
| `publish_rate` | 30 Hz | TF output rate |

The one-command bundle (`limo_mocap_nav2.launch.py`) accepts the same `reg_*`
and `jump_threshold` arguments.

---

## Part 4 — Registration (map ↔ world alignment)

The OptiTrack ground plane and the Cartographer map origin were set at the **same
physical spot**, so `map ≡ world` (identity, all `reg_*` = 0). Measured check: at
the origin OptiTrack reads `x≈0.018, y≈0.054`, yaw ≈ 1.2°.

The final check happens in RViz (Terminal F of the
[daily manual](OPTITRACK_NAV2_DAILY.md#terminal-f--rviz-container)): load the
map and overlay `/scan`. If the laser hits the walls, it's aligned and you're
done. If it's shifted or rotated, set `reg_x/reg_y/reg_yaw` (no rebuild) or
re-run the mapping starting from the OptiTrack world origin.

To estimate the offsets: park the robot, read the mocap pose
(`ros2 topic echo /vrpn_mocap/Limo/pose`), then read where the robot sits on the
map in RViz (hover the cursor over its center). `reg_x`/`reg_y` is the
difference; for yaw, drive a straight line along a wall and compare headings.

---

## Part 5 — Confirm the full chain

With robot bringup + `vrpn_mocap` + `mocap_map_odom` running:

```bash
ros2 run tf2_ros tf2_echo map base_link        # matches the true position, tracks the robot
ros2 run tf2_tools view_frames.py              # writes frames.pdf: map → odom → base_link → sensors
```

Confirmed tree:

```
map → odom (30 Hz, our node) → base_link (50 Hz, robot) → {laser_link +0.18 m, imu, camera}
```

---

## Part 6 — Save the container image

On the **laptop host** (not in the container):

```bash
sudo docker commit limo_laptop limo_foxy:mocap-ready
```

This snapshots the container with `vrpn_mocap`, `netbase` and everything else
installed. To recreate from it later, use the `docker run` from DEVICE_SETUP
step 3 with `limo_foxy:mocap-ready` in place of `osrf/ros:foxy-desktop`. (The
workspace and maps live in the mounts, not the image.)

---

## Watch items (revisit if needed)

- **~9° rigid-body tilt** in the mocap orientation: fine for 2D (yaw only). To
  clean it up, redo the Builder reset with the robot on level ground.
- **Clock sync laptop ↔ LIMO:** they're separate machines, and clock drift causes
  Nav2 TF extrapolation errors. Fix with `chrony`/NTP if it appears.
- **Off-center pivot:** if the reported XY "swings" when the robot rotates in
  place, recenter the Motive pivot over the rotation axis.

---

_Part of the [LIMO documentation index](../README.md#documentation) · [repo home](../README.md)._
