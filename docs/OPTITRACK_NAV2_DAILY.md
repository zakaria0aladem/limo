# Mocap Nav2 — daily manual

**The daily workflow:** drive the LIMO under Nav2 using OptiTrack for absolute
localization. **Compared with the AMCL workflow:** no `bringup_launch.py`, **no
2D Pose Estimate**, no `static_transform_publisher` bootstrap. OptiTrack
localizes the robot automatically; you launch and send goals.

**Prerequisites:** the one-time setup in
[`OPTITRACK_NAV2_SETUP.md`](OPTITRACK_NAV2_SETUP.md) is done. Motive is open on the
PC with the **`Limo`** rigid body tracking and VRPN streaming on. Robot and laptop
are on the same subnet, **domain 10**, **Fast DDS**.

| Terminal | Where | Runs |
|---|---|---|
| A | robot (SSH) | drivers + LiDAR |
| B | container | `vrpn_mocap` (OptiTrack → ROS) |
| C | container | `mocap_map_odom` (replaces AMCL) |
| D | container | `map_server` |
| E | container | Nav2 `navigation_launch.py` (no AMCL) |
| F | container | RViz |

**Shortcut:** C, D and E can be replaced by one command; see
[One-command alternative](#one-command-alternative-replaces-terminals-c-d-e).

---

## Pre-flight (verify IPs; DHCP changes them)

```bash
hostname -I          # on the LIMO      (last seen: 192.168.8.185)
ipconfig             # on the Motive PC (last seen: 192.168.8.184)
```

Use the **Motive PC IP** for `vrpn_mocap server:=`.

---

## Terminal A — Robot drivers (SSH to the LIMO)

```bash
ssh agilex@192.168.8.185
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

Wait for `connect the serial port: '/dev/ttyTHS0'` and `Now lidar is scanning...`.
**Leave running** (closing it stops the robot).

---

## Terminal B — Mocap driver (container)

```bash
sudo docker start limo_laptop
sudo docker exec -it limo_laptop bash
```

```bash
ros2 launch vrpn_mocap client.launch.yaml server:=192.168.8.184 port:=3883
```

Look for `Created new tracker Limo`. **Leave running.**

**Quick sanity check** in a spare container shell: `ros2 topic list` should show
**both** the robot topics (`/odom /scan /tf`) **and** `/vrpn_mocap/Limo/pose`.
If not, it's a DDS/domain mismatch: `ros2 daemon stop && ros2 daemon start` and
re-check the environment.

---

## Terminal C — Mocap localizer (container)

```bash
sudo docker exec -it limo_laptop bash
ros2 launch mocap_localization mocap_localization.launch.py
# with options, e.g.:  ... mocap_localization.launch.py jump_threshold:=0.3
```

It prints `mocap_map_odom: ... -> map->odom`. A single startup "waiting for
odom->base_link" line is normal; if it repeats, the robot's TF isn't visible.
**Leave running.** This is what replaces AMCL.

---

## Terminal D — Map server (container)

```bash
sudo docker exec -it limo_laptop bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/root/maps/mapMTR5.yaml -p use_sim_time:=false
```

In a spare shell, **activate it** (it's a lifecycle node and starts inactive):

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

**Use `navigation_launch.py`, NOT `bringup_launch.py`.** `bringup_launch.py`
starts AMCL, which also publishes `map → odom` and **fights** the mocap node (the
robot's pose jumps between the two). `navigation_launch.py` is planner +
controller + costmaps only; the map is served separately (Terminal D).

---

## One-command alternative (replaces Terminals C, D, E)

Once A and B are up:

```bash
sudo docker exec -it limo_laptop bash
ros2 launch mocap_localization limo_mocap_nav2.launch.py
# or the copy in ~/maps:  ros2 launch /root/maps/limo_mocap_nav2.launch.py
# options: map:=... params_file:=... reg_x:=... reg_y:=... reg_yaw:=... jump_threshold:=...
```

This starts map_server (auto-activated), `mocap_map_odom`, and Nav2 without AMCL.
Nav2 is delayed 3 s so `/map` is already latched when the costmaps come up.

---

## Terminal F — RViz (container)

```bash
sudo docker exec -it limo_laptop bash
rviz2
```

- **Fixed Frame:** `map`
- Add → By topic: **Map** `/map` (Durability: *Transient Local*) · **LaserScan**
  `/scan` (Reliability: *Best Effort*) · **Map** `/global_costmap/costmap`
  (Color Scheme: costmap) · **TF**
- The robot should already appear at its **true position**. No 2D Pose Estimate
  needed (and don't click it: AMCL isn't running).

**Alignment check (once per session):** the laser dots should land on the map's
black walls. **Off?** Stop Terminal C and relaunch it with
`reg_x:=… reg_y:=… reg_yaw:=…` (see
[registration](OPTITRACK_NAV2_SETUP.md#part-4--registration-map--world-alignment)).
Don't touch the map.

---

## Send goals

**RViz:** click **2D Goal Pose**, then click + drag on the map (arrow = final
heading). The robot drives.

**CLI, fire-and-forget** (topic):

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}"
```

**CLI, with feedback + result** (action; blocks until the goal succeeds or fails):

```bash
ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

**MATLAB:** `h = limo_connect(domainID=10); limo_goal(h, 1.0, 0.5, 0, wait=true)`,
or a whole route with `send_route_goalpose.m` (see
[`OPTITRACK_NAV2_PROJECT.md`](OPTITRACK_NAV2_PROJECT.md#phase-5--goal-sending)).

---

## Troubleshooting

| Symptom | Likely cause → fix |
|---|---|
| Container can't see robot topics | DDS/domain. `ros2 daemon stop && ros2 daemon start`; check `echo $RMW_IMPLEMENTATION` = `rmw_fastrtps_cpp` and `$ROS_DOMAIN_ID` = 10; if `$FASTRTPS_DEFAULT_PROFILES_FILE` is set, that file must exist |
| `vrpn_mocap` "connection is bad" + `getprotobyname failed` | `apt install -y netbase`, relaunch |
| No `/vrpn_mocap/Limo/pose` | Motive VRPN off / firewall on port 3883 / wrong Local Interface IP / rigid body not named `Limo` |
| `mocap_map_odom` spams "waiting for odom->base_link" | Terminal A not up or not visible; confirm `ros2 run tf2_ros tf2_echo odom base_link` works |
| Robot pose offset or rotated in RViz | Registration: relaunch Terminal C with `reg_x/reg_y/reg_yaw` |
| Pose jumps around | Mocap flips → relaunch Terminal C with `jump_threshold:=0.3` |
| Pose jumps between two places | AMCL is also running (`bringup_launch.py` or a leftover static transform); stop it |
| Map doesn't show in RViz | Map display Durability = *Transient Local*; is map_server active? (see below) |
| Robot won't move on a goal | `ros2 topic echo /cmd_vel` (is Nav2 publishing?); try a closer goal; check the costmap isn't fully inflated over the robot |
| Robot stops a few cm short | That's `xy_goal_tolerance` in `nav2.yaml`, by design |
| `Deserialization of data failed` | `limo_msgs` not sourced: `source /root/ros2_ws/install/setup.bash` |
| Nav2 TF extrapolation errors | Laptop↔LIMO clock drift → set up `chrony`/NTP |

### map_server lifecycle

Check its state:

```bash
ros2 lifecycle get /map_server
```

Healthy = `active [3]`.

The states (and allowed moves):

```
unconfigured --configure--> inactive --activate--> active
   active --deactivate--> inactive          active --shutdown--> finalized
```

You can only make a move that's valid *from the current state*. `Unknown
transition requested, available: deactivate, shutdown` just means it's already
active.

**Bounce it to re-publish the latched `/map`** (the fix when a subscriber missed it):

```bash
ros2 lifecycle set /map_server deactivate
ros2 lifecycle set /map_server activate
```

**Activate it manually if it never came up** (state shows `unconfigured`):

```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

---

## Shutdown (reverse order)

1. **F** RViz: close
2. **E** Nav2: `Ctrl+C` (wait for a clean exit)
3. **D** map_server: `Ctrl+C`
4. **C** mocap_map_odom: `Ctrl+C`
5. **B** vrpn_mocap: `Ctrl+C`
6. **A** robot: `Ctrl+C`
7. Container: leave it running, or `sudo docker stop limo_laptop` on the host

---

## Quick reference

| What | Value / command |
|---|---|
| Domain / DDS | `10` / `rmw_fastrtps_cpp` |
| Mocap topic | `/vrpn_mocap/Limo/pose` (100 Hz, frame `world`) |
| Localizer node | `mocap_map_odom` → publishes `map→odom` @ 30 Hz |
| Map files | `/root/maps/mapMTR5.{yaml,pgm}`, `/root/maps/nav2.yaml` |
| Steering / controller | Differential / DWB |
| Verify localization | `ros2 run tf2_ros tf2_echo map base_link` |
| Goal topic / action | `/goal_pose` · `/navigate_to_pose` |

**Terminal map:** A robot (SSH) · B vrpn_mocap · C mocap_map_odom · D map_server ·
E navigation_launch · F rviz2. B–F run inside the container.

## References

- [Mocap first-time setup](OPTITRACK_NAV2_SETUP.md) ·
  [project log](OPTITRACK_NAV2_PROJECT.md) · [device setup](DEVICE_SETUP.md)

---

_Part of the [LIMO documentation index](../README.md#documentation) · [repo home](../README.md)._
