# Mocap Nav2 — Daily Manual

> [!abstract] Daily workflow: drive the LIMO under Nav2 using OptiTrack for absolute localization. **vs the old AMCL workflow:** no `bringup_launch.py`, **no 2D Pose Estimate**, no `static_transform_publisher` bootstrap. OptiTrack localizes the robot automatically and absolutely. You just launch and send goals.

> [!info] Prereqs One-time setup done (Mocap Nav2 — First Setup). Motive open on the PC with the **`Limo`** rigid body tracking + VRPN streaming on. Robot and laptop on the same subnet, **domain 10**, **Fast DDS**.

---

## Pre-flight (verify IPs — DHCP changes them)

```bash
hostname -I          # on LIMO  (this session: 192.168.8.185)
ipconfig             # on Motive PC (this session: 192.168.8.184)
```

Use the **Motive PC IP** for `vrpn_mocap server:=`.

---

## Terminal A — Robot drivers (SSH to LIMO)

```bash
ssh agilex@192.168.8.185
```

```bash
pkill -9 ros2
```

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
```

```bash
ros2 launch limo_bringup limo_start.launch.py
```

Wait for: `connect the serial port: '/dev/ttyTHS0'` and `Now lidar is scanning...`. **Leave running** (closing it stops the robot).

---

## Terminal B — Mocap driver (container)

```bash
sudo docker start limo_laptop
sudo docker exec -it limo_laptop bash
ros2 launch vrpn_mocap client.launch.yaml server:=192.168.8.184 port:=3883
```

Look for: `Created new tracker Limo`. **Leave running.**

> [!tip] Quick sanity in a spare shell `ros2 topic list` should show **both** robot topics (`/odom /scan /tf`) **and** `/vrpn_mocap/Limo/pose`. If not → DDS/domain mismatch (`ros2 daemon stop && ros2 daemon start`).

---

## Terminal C — Mocap localizer (container)

```bash
sudo docker exec -it limo_laptop bash
ros2 launch mocap_localization mocap_localization.launch.py
```

Prints `mocap_map_odom: ... -> map->odom`. A single startup "waiting for odom->base_link" line is normal; repeating = robot TF not visible. **Leave running.** This is what replaces AMCL.

---

## Terminal D — Map server (container)

```bash
sudo docker exec -it limo_laptop bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/root/maps/mapMTR5.yaml -p use_sim_time:=false
```

In a spare shell, **activate it** (lifecycle node):

```bash
ros2 run nav2_util lifecycle_bringup map_server
```

`/map` is now published. **Leave running.**

---

## Terminal E — Nav2 navigation, NO AMCL (container)

```bash
sudo docker exec -it limo_laptop bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=/root/maps/nav2.yaml use_sim_time:=false
```

Wait for `Managed nodes are active`. **Leave running.**

> [!warning] Use `navigation_launch.py`, NOT `bringup_launch.py` `bringup_launch.py` starts AMCL, which also publishes `map→odom` and **fights** the mocap node. `navigation_launch.py` is planner+controller+costmaps only. Map is served separately (Terminal D).

---

## Terminal F — RViz (container)

```bash
sudo docker exec -it limo_laptop bash
rviz2
```

- **Fixed Frame:** `map`
- Add → by topic: **Map** `/map` (Durability: _Transient Local_) · **LaserScan** `/scan` (Reliability: _Best Effort_) · **Map** `/global_costmap/costmap` (Color: costmap) · **TF**
- The robot should already appear at its **true position** — no 2D Pose Estimate needed.

> [!tip] Alignment check (do once per session) Laser dots should land on the map's black walls. **Off?** Stop Terminal C, edit `reg_x/reg_y/reg_yaw` in the launch file, relaunch. Don't touch the map.

---

## Send goals

**RViz:** click **2D Goal Pose** → click+drag on the map (arrow = final heading). Robot drives.

**CLI:**

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}"
```

**Action (feedback + result):**

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}"
```

---

## Troubleshooting

|Symptom|Likely cause → fix|
|---|---|
|Container can't see robot topics|DDS/domain. `ros2 daemon stop && ros2 daemon start`; check `echo $RMW_IMPLEMENTATION` = `rmw_fastrtps_cpp`, `$ROS_DOMAIN_ID` = 10; `unset FASTRTPS_DEFAULT_PROFILES_FILE`|
|`vrpn_mocap` "connection is bad" + `getprotobyname failed`|`apt install -y netbase`, relaunch|
|No `/vrpn_mocap/Limo/pose`|Motive VRPN off / firewall on 3883 / wrong Local Interface IP / rigid body not named `Limo`|
|`mocap_map_odom` spams "waiting for odom->base_link"|Terminal A not up / not visible; confirm `tf2_echo odom base_link` works|
|Robot pose offset or rotated in RViz|Registration: set `reg_x/reg_y/reg_yaw` in mocap launch|
|Pose jumps around|Mocap flips → set `jump_threshold:=0.3` in mocap launch|
|Map doesn't show in RViz|Map display Durability = _Transient Local_; map_server activated?; `ros2 lifecycle set /map_server deactivate && ... activate`|
|Robot won't move on goal|`ros2 topic echo /cmd_vel` (is Nav2 publishing?); try a closer goal; check costmap not fully inflated over robot|
|`Deserialization of data failed`|`limo_msgs` not sourced: `source /root/ros2_ws/install/setup.bash`|
|Nav2 TF extrapolation errors|Laptop↔LIMO clock drift → set up `chrony`/NTP|
**Check its state:**

bash

```bash
ros2 lifecycle get /map_server
```

✓ healthy = `active [3]`.

**The states (and allowed moves):**

```
unconfigured --configure--> inactive --activate--> active
   active --deactivate--> inactive          active --shutdown--> finalized
```

You can only make a move that's valid _from the current state_.

**Bounce it to re-publish the latched `/map`** (the fix when a subscriber missed it):

bash

```bash
ros2 lifecycle set /map_server deactivate
ros2 lifecycle set /map_server activate
```

**Activate it manually if it never came up** (state shows `unconfigured`):

bash

```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```
---

## Shutdown (reverse order)

1. **F** RViz — close
2. **E** Nav2 — Ctrl+C (wait for clean exit)
3. **D** map_server — Ctrl+C
4. **C** mocap_map_odom — Ctrl+C
5. **B** vrpn_mocap — Ctrl+C
6. **A** robot — Ctrl+C
7. Container: leave running, or `sudo docker stop limo_laptop`

---

## Quick reference

|What|Value / command|
|---|---|
|Domain / DDS|`10` / `rmw_fastrtps_cpp`|
|Mocap topic|`/vrpn_mocap/Limo/pose` (100 Hz, frame `world`)|
|Localizer node|`mocap_map_odom` → publishes `map→odom` @30 Hz|
|Map files|`/root/maps/mapMTR5.{yaml,pgm}`, `/root/maps/nav2.yaml`|
|Steering / controller|Differential / DWB|
|Verify localization|`ros2 run tf2_ros tf2_echo map base_link`|
|Goal topic / action|`/goal_pose` · `/navigate_to_pose`|

> [!note] Terminal map A robot(SSH) · B vrpn_mocap · C mocap_map_odom · D map_server · E navigation_launch · F rviz2 — all B–F inside the container.

## References

- Mocap Nav2 First Setup
- Limo robot · setup your device
