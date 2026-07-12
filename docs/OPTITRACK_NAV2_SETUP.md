# Mocap Absolute Positioning — First-Time Setup

> [!abstract] What this adds Replaces Nav2's AMCL (lidar guess + manual **2D Pose Estimate**) with **OptiTrack absolute positioning**. After this, the robot knows its true position automatically — no pose initialization, no drift. A custom node publishes `map → odom` from the mocap pose; the rest of Nav2 is unchanged.

> [!info] One-time only Do this **once**. Daily running lives in Mocap Nav2 — Daily Manual. Finish by saving a container image so you never repeat it.

---

## Prerequisites (already done elsewhere)

These come from setup your device — don't redo them:

- Foxy container `limo_laptop` (`osrf/ros:foxy-desktop`), `--net=host`, domain **10**, **`rmw_fastrtps_cpp`**, mounts `~/ros2_ws` + `~/maps`.
- `limo_msgs` built in the container (or "Deserialization of data failed").
- Nav2 installed: `ros-foxy-navigation2 ros-foxy-nav2-bringup`.
- Map files in `/root/maps/`: `mapMTR5.yaml`, `mapMTR5.pgm`, `nav2.yaml`.

> [!warning] DDS — use Fast DDS, not Cyclone Everything in this project uses **`rmw_fastrtps_cpp`** on **domain 10**, on both robot and container. (Older notes contain one stray `rmw_cyclonedds_cpp` line — ignore it.) A mismatch makes Nav2 flaky.

---

## Part 1 — OptiTrack / Motive (Windows PC)

The Motive PC and LIMO must share a subnet.

> [!note] IPs are DHCP — verify each session This session: **Motive PC `192.168.8.184`** (wired), **LIMO `192.168.8.185`** (WiFi). Confirm with `ipconfig` (PC) and `hostname -I` (LIMO) before relying on them.

**Steps in Motive:**

1. Confirm the capture volume is **calibrated** (status panel). If not, run wand calibration + ground plane.
2. Stick **≥4 markers** on the LIMO in an **asymmetric, varied-height** pattern (avoid squares/lines → prevents flips).
3. Select the markers → right-click → **Rigid Body → Create From Selected**. Rename to **`Limo`** (capital L, **no spaces** — VRPN drops names with spaces). Streaming ID `5`.
4. Fix orientation: **Builder pane → Rigid Bodies → Edit** — square the robot to a world axis, then **Reset Orientation**.
5. Rigid-body tuning (Properties): **Forward Prediction → 0** (200 caused flips at speed/stops), **Smoothing 0**, **Min Marker Count 3**, **Max Deflection ~5–20 mm**.
6. **Data Streaming pane:** enable streaming · **Local Interface = `192.168.8.184`** · **Up Axis = Z Up** (matches ROS) · Stream Rigid Bodies On · **VRPN On, port `3883`**.

> [!bug] Why these settings (hard-won)
> 
> - **Forward Prediction 0** — extrapolation fights a slow robot; flips on sudden stops.
> - **Up Axis Z** — ROS is Z-up (REP-103); Motive defaults Y-up.
> - **Min Marker Count 3** — survives one occluded marker.
> - Residual flips/drops in some arena corners = camera coverage gaps. Accepted; handled downstream (`jump_threshold`).

---

## Part 2 — Install the VRPN driver (container)

```bash
docker exec -it limo_laptop bash
apt update && apt install -y ros-foxy-vrpn-mocap netbase
```

> [!bug] `getprotobyname() failed` / "VRPN connection is bad" The minimal container lacks `/etc/protocols`, so VRPN can't open its UDP socket. **`netbase`** provides the file. **Always install `netbase` on a fresh container.** (Harmless: a `vrpn ver 07.35 vs 07.33` mismatch warning.)

**Test:**

```bash
ros2 launch vrpn_mocap client.launch.yaml server:=192.168.8.184 port:=3883
# second shell:
ros2 topic echo /vrpn_mocap/Limo/pose      # numbers move when robot moves
ros2 topic hz   /vrpn_mocap/Limo/pose      # ~100 Hz
```

Topic is **`/vrpn_mocap/Limo/pose`** (`geometry_msgs/PoseStamped`, frame `world`).

---

## Part 3 — Install the `mocap_localization` package (container)

This package's `mocap_map_odom` node publishes `map → odom` from the mocap pose, replacing AMCL.

```bash
# put install_mocap_localization.sh in ~/ros2_ws (mounted), then:
cd /root/ros2_ws
bash install_mocap_localization.sh
colcon build --packages-select mocap_localization --symlink-install
source install/setup.bash
```

**The math it implements:**

```
map→odom = (map→base) ∘ (odom→base)⁻¹
```

`map→base` = mocap pose (2D); `odom→base` = robot wheel odometry (left untouched). The node absorbs all drift into `map→odom` — exactly AMCL's contract.

**Tunable params** (in `launch/mocap_localization.launch.py`):

|Param|Default|Use|
|---|---|---|
|`reg_x`,`reg_y`,`reg_yaw`|0|Correct map↔world misalignment **without** rebuilding the map|
|`jump_threshold`|0 (off)|e.g. `0.3` → reject mocap flips/dropouts (meters)|
|`publish_rate`|30 Hz|TF output rate|

---

## Part 4 — Registration (map ↔ world alignment)

Because the OptiTrack ground plane and the Cartographer map origin were set to the **same physical spot**, `map ≡ world` (identity, `reg=0`). Verified: at the origin OptiTrack reads `x≈0.018, y≈0.054`, yaw ≈ 1.2°.

> [!tip] Final check happens in RViz (Part 5 of daily manual) Load the map, overlay `/scan`. If laser hits the walls → aligned, done. If shifted/rotated → set `reg_x/reg_y/reg_yaw` (no rebuild) or re-run Cartographer from the world origin.

---

## Part 5 — Confirm the full chain

With robot bringup + `vrpn_mocap` + `mocap_map_odom` running:

```bash
ros2 run tf2_ros tf2_echo map base_link        # matches true position, tracks robot
ros2 run tf2_tools view_frames.py              # map → odom → base_link → sensors
```

Confirmed tree:

```
map → odom (30 Hz, our node) → base_link (50 Hz, robot) → {laser_link +0.18m, imu, camera}
```

---

## Part 6 — Save the container image

On the **laptop host** (not container):

```bash
sudo docker commit limo_laptop limo_foxy:mocap-ready
```

Snapshots the container with `vrpn_mocap`, `netbase`, and `mocap_localization` all installed. Recreate anytime without redoing Parts 2–3.

---

## Watch-items (revisit if needed)

- **~9° rigid-body tilt** in mocap orientation — fine for 2D (yaw only); to clean, redo Builder reset on level ground.
- **Clock sync laptop↔LIMO** — separate machines; drift → Nav2 TF extrapolation errors. Fix with `chrony`/NTP if it appears.
- **Pivot off-center** — if reported XY "swings" when rotating in place, recenter the Motive pivot over the rotation axis.

## References

- setup your device — container, limo_msgs, Nav2 install
- Mocap Nav2 — Daily Manual — everyday run
- LIMO_OptiTrack_Nav2_Setup — full phase-by-phase log
- OptiTrack Motive · vrpn_mocap (alvinsunyixiao) · docs.nav2.org
