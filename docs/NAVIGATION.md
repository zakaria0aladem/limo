# Navigation with AMCL on a saved map (Nav2)

Autonomous navigation with the onboard LiDAR: load a saved map, localize with
**AMCL** (LiDAR-to-wall matching, seeded by a manual **2D Pose Estimate**), then
send goals. This is the manufacturer's standard workflow, plus a laptop-side
variant and Nav2 speed tuning.

**Prerequisites:** the one-time base setup in [`DEVICE_SETUP.md`](DEVICE_SETUP.md),
and a saved map. The map in this repo (`config/mapMTR5.{yaml,pgm}`) covers the
OptiTrack area of the MTR lab (ESB 0012). To map a different space, build one
first with [`SLAM.md`](SLAM.md) and save it in `~/maps` (`/root/maps` in the
container).

**Contents:** [Network](#network) · [Option A: onboard](#option-a--run-everything-on-the-robot-vendor-launch) ·
[Option B: laptop](#option-b--nav2-on-the-laptop-daily-workflow) ·
[Troubleshooting](#troubleshooting) · [Speed tuning](#nav2-speed-tuning)

---

## Network

Both machines must be on the same network **and** subnet, on the same ROS 2
distribution (Foxy), with `ROS_DOMAIN_ID=10` and `rmw_fastrtps_cpp`. IPs are
DHCP, so check the robot's IP every session (on the LIMO):

```bash
hostname -I
```

Known networks (IPs last seen; verify):

| Network | LIMO IP | SSH |
|---|---|---|
| `AUS_Wireless` | `10.25.150.233` | `ssh agilex@10.25.150.233` |
| `GL-BE9300-2a5` (lab router) | `192.168.8.185` | `ssh agilex@192.168.8.185` |

Use `ssh -X ...` if you want robot-side windows (RViz on the robot) to display
on the laptop.

**Every new robot shell** (and every container shell, unless your `~/.bashrc`
already sets these) needs:

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE      # robot only; see DEVICE_SETUP step 5 for the container
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
```

If the laptop can't see robot topics and everything above matches, a firewall
may be blocking DDS traffic. Temporarily test with `sudo ufw disable` on the
laptop (re-enable afterwards with `sudo ufw enable`).

---

## Option A — Run everything on the robot (vendor launch)

Quickest check that navigation works; everything runs on the LIMO and uses the
map configured in AgileX's `limo_bringup`.

1. SSH in, clean stale processes, set the environment (above):

   ```bash
   ssh agilex@192.168.8.185
   pkill -9 ros2
   ```

2. Launch the LiDAR and base drivers:

   ```bash
   ros2 launch limo_bringup limo_start.launch.py
   ```

3. In a second SSH shell, start navigation (with RViz over `ssh -X`, or headless):

   ```bash
   ros2 launch limo_bringup limo_nav2.launch.py
   # or, no RViz:
   ros2 launch limo_bringup limo_nav2.launch.py use_rviz:=false
   ```

4. Set the initial pose in RViz (**2D Pose Estimate**), then send a goal:

   ```bash
   ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
     "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}"
   ```

---

## Option B — Nav2 on the laptop (daily workflow)

The robot only runs its drivers; Nav2, AMCL, the map and RViz run in the laptop
container with this repo's `nav2.yaml` and map. This is the setup the mocap
workflow later builds on.

| Terminal | Where | Runs |
|---|---|---|
| A | robot (SSH) | drivers + LiDAR |
| B | container | Nav2 with AMCL + map_server (`bringup_launch.py`) |
| C | container | temporary `map → odom` bootstrap |
| D | container | RViz |

### Step 1 — Robot drivers (Terminal A)

```bash
ssh agilex@192.168.8.185        # or whichever IP the robot has today
pkill -9 ros2
```

Set the environment block from [Network](#network), then:

```bash
ros2 launch limo_bringup limo_start.launch.py
```

Wait until you see:

- `[limo_base] connect the serial port: '/dev/ttyTHS0'`
- `[YDLIDAR] Now lidar is scanning...`

**Leave this terminal running.** Closing it stops the robot.

### Step 2 — Enter the container and check the link (Terminal B)

On the laptop host:

```bash
xhost +local:docker
sudo docker start limo_laptop
sudo docker exec -it limo_laptop bash
```

Inside the container:

```bash
ros2 daemon stop && ros2 daemon start
ros2 topic list
```

Expected: `/cmd_vel`, `/imu`, `/limo_status`, `/odom`, `/scan`, `/tf`, `/tf_static`.
If they're missing, see [Troubleshooting](#troubleshooting) before going on.

### Step 3 — Launch Nav2 with AMCL (Terminal B)

```bash
ros2 launch nav2_bringup bringup_launch.py \
    map:=/root/maps/mapMTR5.yaml \
    params_file:=/root/maps/nav2.yaml \
    use_sim_time:=false
```

Wait for `Managed nodes are active` (about 10 s). **Leave running.**

### Step 4 — Bootstrap the map frame (Terminal C)

Until you give AMCL an initial pose it publishes no `map → odom`, so RViz can't
place the robot on the map. Publish a temporary identity transform:

```bash
sudo docker exec -it limo_laptop bash
```

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

**Leave it running for now; you'll stop it in Step 7.**

### Step 5 — Open RViz (Terminal D)

```bash
sudo docker exec -it limo_laptop bash
rviz2
```

In RViz:

1. **Global Options → Fixed Frame:** `map`
2. **Add → By topic:**
    - `/map` → Map (**Durability Policy: Transient Local**)
    - `/scan` → LaserScan (**Reliability: Best Effort**)
    - `/global_costmap/costmap` → Map (**Durability Policy: Transient Local**, **Color Scheme: costmap**)
3. The map should be visible.

### Step 6 — Localize the robot

1. Find where the robot physically is in the room on the map.
2. Click **2D Pose Estimate** at the top of RViz.
3. **Click + drag** at the robot's real position, arrow pointing the way the robot faces.
4. Release.

AMCL now publishes the real `map → odom`. The laser dots should line up with the
black walls. If not, click 2D Pose Estimate again and refine; the heading matters
more than the position.

### Step 7 — Stop the bootstrap transform (Terminal C)

Press `Ctrl+C` in Terminal C. Two publishers of `map → odom` fight each other, so
this step is not optional. The map should stay put; if it jumps, repeat Step 6.

### Step 8 — Send navigation goals

**RViz:** click **2D Goal Pose**, then click + drag on the map at the destination
(arrow = final heading). The robot drives.

**Command line** (any container shell):

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}"
```

Keep `--once`: without it, `ros2 topic pub` re-sends the goal every second.
YAML needs a space after every colon (`frame_id: 'map'`, not `frame_id:'map'`).

### Shutdown sequence

1. **Terminal D (RViz):** close the window or `Ctrl+C`.
2. **Terminal B (Nav2):** `Ctrl+C`, wait until all nodes shut down.
3. **Terminal A (robot):** `Ctrl+C`.
4. The container can stay running, or stop it from the laptop host:
   `sudo docker stop limo_laptop`.

---

## Troubleshooting

**Topics not appearing in the container**

```bash
ros2 daemon stop && ros2 daemon start
echo $RMW_IMPLEMENTATION    # must be rmw_fastrtps_cpp
echo $ROS_DOMAIN_ID         # must be 10
echo $FASTRTPS_DEFAULT_PROFILES_FILE   # if set, the file must exist: ls "$FASTRTPS_DEFAULT_PROFILES_FILE"
```

Also check both machines' IPs are on the same subnet, and see the firewall note
under [Network](#network).

**Map doesn't show in RViz**

- The Map display's **Durability Policy** must be `Transient Local`.
- Fixed Frame must be `map` (keep the Step 4 bootstrap running until AMCL has a pose).
- Force a republish:
  `ros2 lifecycle set /map_server deactivate && ros2 lifecycle set /map_server activate`

**Robot won't move on a goal**

- Is Nav2 publishing? `ros2 topic echo /cmd_vel`
- Does AMCL have a valid pose? Laser dots should overlap the walls.
- Is the goal reachable? Try a closer, simpler goal in open white space.
- Is something else publishing `/cmd_vel` (wandering node, Simulink)? Only one may run.

**`Deserialization of data failed`**

`limo_msgs` isn't sourced in this shell:
`source /root/ros2_ws/install/setup.bash` (see DEVICE_SETUP step 5).

**Robot pose jumps or the map slides**

The `static_transform_publisher` from Step 4 is still running alongside AMCL.
Stop it.

---

## Nav2 speed tuning

Velocity limits live in `nav2.yaml` under `controller_server` (DWB controller).
**Restart Nav2 after editing.**

There are two speed limits:

1. Robot hardware max (~1.0 m/s for the LIMO).
2. Nav2-allowed max (set in `nav2.yaml`). This is what actually limits you.

### Parameters that matter

| Param | Default in repo | Purpose |
|---|---|---|
| `max_vel_x` | 0.22 | top linear speed (m/s) |
| `max_speed_xy` | 0.44 | speed magnitude cap (must be ≥ `max_vel_x`) |
| `max_vel_theta` | 0.8 | top angular speed (rad/s) |
| `decel_lim_x` | -0.5 | braking; must scale with speed |
| `acc_lim_theta` | 0.2 | angular acceleration |
| `controller_frequency` | 10.0 | control loop rate (Hz) |
| `inflation_radius` | 0.02 | wall clearance buffer (costmap, both costmaps) |
| `sim_time` | 1.5 | DWB trajectory look-ahead time (s) |
| `xy_goal_tolerance` | 0.05 | goal arrival tolerance (m) |

Always check the current values first. The `sed` commands below only match the
exact "from" value:

```bash
grep -E "max_vel_x:|max_speed_xy:|decel_lim_x:|controller_frequency:|inflation_radius:|sim_time:|xy_goal_tolerance:" /root/maps/nav2.yaml
```

Example: a faster profile (edit the "from" values to match the grep output):

```bash
cp /root/maps/nav2.yaml /root/maps/nav2.yaml.bak          # keep a way back
sed -i 's/max_vel_x: 0.22/max_vel_x: 0.80/' /root/maps/nav2.yaml
sed -i 's/max_speed_xy: 0.44/max_speed_xy: 0.80/' /root/maps/nav2.yaml
sed -i 's/decel_lim_x: -0.5/decel_lim_x: -2.0/' /root/maps/nav2.yaml
sed -i 's/controller_frequency: 10.0/controller_frequency: 20.0/' /root/maps/nav2.yaml
sed -i 's/inflation_radius: 0.02/inflation_radius: 0.25/' /root/maps/nav2.yaml
sed -i 's/sim_time: 1.5/sim_time: 2.0/' /root/maps/nav2.yaml
sed -i 's/xy_goal_tolerance: 0.05/xy_goal_tolerance: 0.15/' /root/maps/nav2.yaml
```

Then run the `grep` again to confirm. These edits change `~/maps/nav2.yaml` only;
copy it back to `~/limo/config/` if you want the repo to keep them.

### Rules and warnings

- **Scale supporting params with speed.** Raising `max_vel_x` alone makes the
  robot lurch and overshoot. Deceleration, controller frequency, inflation radius
  and `sim_time` must all scale up too.
- **Hardware limit ~1.0 m/s.** Above that the motors saturate. A safe indoor
  ceiling is ~0.6–0.7 m/s; 0.8 is aggressive.
- **Turning radius** = `max_vel_x / max_vel_theta`. At 0.8 / 0.8 that's 1.0 m. If
  the robot can't make a corner, raise `max_vel_theta`.
- **`sed` only replaces exact matches.** If a command "does nothing", the value
  was already changed. `grep` first, then target those exact numbers.

### Measure actual speed

Send a goal down a long straight path and watch the reported velocity. Foxy's
`ros2 topic echo` has no `--field` option, so filter the output instead:

```bash
ros2 topic echo /odom | grep -A1 "linear:"
```

The peak `x:` value is the real top speed.

---

## References

- MathWorks, *ROS Toolbox Support Package for TurtleBot-Based Robots* user guide
  (R2023a): <https://www.mathworks.com/help/releases/R2023a/pdf_doc/supportpkg/turtlebotrobot/turtlebotrobot_ug.pdf>
- Nav2 documentation: <https://docs.nav2.org>

---

_Part of the [LIMO documentation index](../README.md#documentation) · [repo home](../README.md)._
