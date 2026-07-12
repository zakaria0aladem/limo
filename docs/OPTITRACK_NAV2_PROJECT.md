# LIMO Pro + Nav2 + OptiTrack — Absolute Positioning Setup

> [!success] Status: Phases 1–5 COMPLETE · Phase 6 (comparison) in progress OptiTrack → ROS → mocap localizer (`map→odom`) → Nav2 → robot drives waypoint routes. Code packaged as the `mocap_localization` GitHub repo. Now collecting comparison data (mocap vs AMCL vs odom-only).

> [!abstract] What this project is Integrate an **AgileX LIMO Pro** (ROS 2 Foxy) with **Nav2**, using **OptiTrack** motion capture as an **absolute** localization source. A **Cartographer** map already exists. Final phase compares OptiTrack ground truth against onboard estimates.

---

## 📌 Quick Reference

|Item|Value|
|---|---|
|Robot|AgileX LIMO Pro (Jetson Orin Nano, Ubuntu 20.04)|
|ROS distro|**Foxy** (⚠️ EOL since May 2023)|
|Mocap driver|**`vrpn_mocap`** (runs on ARM; NatNet has no ARM build)|
|Motive PC IP (wired)|`192.168.8.184`|
|LIMO IP (WiFi)|`192.168.8.185`|
|Rigid body name|**`Limo`** (capital L, no spaces) · Streaming ID `5` · 4 markers|
|VRPN port|`3883` · Motive Up Axis = **Z Up**|
|Mocap topic|**`/vrpn_mocap/Limo/pose`** (`PoseStamped`, frame `world`, 100 Hz)|
|**DDS / domain**|**`rmw_fastrtps_cpp`**, `ROS_DOMAIN_ID=10` (both machines)|
|Container|`limo_laptop` (`osrf/ros:foxy-desktop`), `--net=host`, mounts `~/ros2_ws` `~/maps`|
|**Moving base frame**|**`base_link`** (parent `odom`, 50 Hz) — _no_ `base_footprint`|
|Laser frame|`laser_link` (+0.180 m x, static)|
|Steering / controller|**Differential / DWB**|
|Map files|`/root/maps/mapMTR5.{yaml,pgm}` · `/root/maps/nav2.yaml`|
|One-command launch|`ros2 launch /root/maps/limo_mocap_nav2.launch.py`|
|Verify localization|`ros2 run tf2_ros tf2_echo map base_link`|

> [!important] `~/maps` (laptop) ↔ `/root/maps` (container) are the **same folder**. Anything else in the container is invisible to the laptop.

---

## 🗺️ The Six Phases

1. **OptiTrack streaming** — Motive tracks the LIMO, streams over VRPN; robot receives it as a ROS topic.
2. **Coordinate frames (TF)** — Align mocap pose with the map; build `map → odom → base_link`.
3. **Localization source** — Replace AMCL's LiDAR guess with OptiTrack's absolute pose.
4. **Nav2 bring-up** — Navigation stack on the existing map; drive to a goal.
5. **Goal sending** — Command "go to X" programmatically.
6. **Comparison** — Mocap vs AMCL vs odom-only, against OptiTrack ground truth.

> [!info] Two facts that shaped the whole plan
> 
> - **The LIMO's CPU is ARM.** OptiTrack's NatNet binary has **no ARM build** → use **`vrpn_mocap`** (VRPN-only, open source).
> - **Foxy is EOL.** Most docs target Humble/Jazzy; expect to back-port.

---

## 📖 Concept Primer — `map → odom → base_link`

> [!info] Terminology **Frame** = a point of view (position + facing). **Transform** = the offset converting between frames (the "→"). **TF** = ROS's frame bookkeeping.

- **`base_link` = the robot itself.** Sensors are fixed offsets from it.
- **`map` = the room.** Fixed; goals and walls live here.
- **`odom` = smooth wheel-based tracker that drifts.** 50 Hz, never jumps, slowly wrong.

**Why two hops:** Nav2 wants `map → base_link` (robot in the room). No direct sensor gives it, so:

```
map → base_link  =  map → odom   +   odom → base_link
(true position)     (correction)     (drifted odometry)
```

- `odom → base_link` = odometry (robot publishes it).
- `map → odom` = **the correction**. Our node publishes it, 30×/s: `map→odom = (map→base) ∘ (odom→base)⁻¹`

**Who corrects whom:** mocap (truth) corrects odom (drifts). Not the reverse.

**Why keep odom if mocap is absolute?**

1. **Smoothness** — interpolates between discrete mocap frames.
2. **Dropout survival** — when mocap drops in an arena dead-spot, last correction holds and odom carries the robot through.
3. **Convention** — REP-105 expects a continuous `odom → base_link`.

> [!tip] Mental model: GPS + step counter **mocap = GPS** (absolute, discrete, drops in tunnels). **odom = step counter** (smooth, continuous, drifts). **`map→odom` = the GPS correction** applied to the step-counter estimate.

**What AMCL was:** the default localizer — _guesses_ position by matching LiDAR to map walls (particle filter), needs manual 2D Pose Estimate, few-cm accurate. Our node _knows_ the pose instead. Both publish the identical `map→odom`, so Nav2 can't tell the difference — that's why the swap is clean.

---

## Phase 1 — OptiTrack Streaming → ROS Topic 

**Result:** `/vrpn_mocap/Limo/pose` live at ~100 Hz, tracks the robot.

### Key rigid-body settings

|Setting|Value|Note|
|---|---|---|
|Min Marker Count|**3**|Survives 1 occluded marker (of 4).|
|Max Deflection|**~5–20 mm**|Tolerance, _not_ noise control. 0 caused dropouts.|
|Smoothing|**0** (1–2 if jittery)|Adds latency; keep low.|
|Forward Prediction|**0**|200 caused flips at speed/sudden stops.|

### Decisions & why

- **`vrpn_mocap`** — only driver that runs natively on ARM.
- **Up Axis = Z Up** — ROS is Z-up (REP-103); Motive defaults Y-up.
- **Forward Prediction → 0** — extrapolation overshoots on a slow robot.
- **Uniform camera exposure/threshold** — mismatched settings make triangulation disagree.

### Troubleshooting

> [!bug] Flipping vs dropping **Flipping** = pattern too symmetric → asymmetric, varied-height markers. (Worse than dropping: sends _confidently wrong_ heading.) **Dropping** = markers lost → spread out, mount high, ≥2–3 cm apart. **Flips at speed/sudden stops** = Forward Prediction 200 → 0. **Residual flips in arena dead-spots** = camera coverage gaps. Accepted; use `jump_threshold`.

> [!bug] `getprotobyname() failed` / "VRPN connection is bad" Container lacks `/etc/protocols`. **Fix: `apt install -y netbase`.** Recurs on every fresh container. (Harmless: `vrpn ver 07.35 vs 07.33` warning.)

---

## Phase 2 — Coordinate Frames / TF 

**Result:** `map → odom (30 Hz) → base_link (50 Hz) → {laser, imu, camera}`. `tf2_echo map base_link` matches the true position.

### Registration

Assumed **`map ≡ world`** (identity). Evidence: at the origin, mocap read `x=0.018, y=0.054`, yaw ≈ 1.2°.

- Node exposes `reg_x/reg_y/reg_yaw` to correct misalignment **without rebuilding the map**.
- Backup: re-run Cartographer starting at the OptiTrack origin → makes `map ≡ world` by construction.

### Measured odom drift (why mocap matters)

Drove 134 cm → odom reported **127 cm** + ~7.5° heading drift on a "straight" run.

### Foxy tooling gotchas

- `ros2 topic echo --once` / `--field` → **not supported in Foxy**. Echo + Ctrl+C.
- `view_frames` → needs `.py`: `ros2 run tf2_tools view_frames.py`.
- No `base_footprint` exists — target `base_link`.

---

## Phase 3 — Mocap as Localization Source 

Custom package **`mocap_localization`**, node **`mocap_map_odom`**, runs in the container. Subscribes `/vrpn_mocap/Limo/pose` → publishes `map→odom` TF at 30 Hz. **AMCL is not launched.**

|Param|Default|Use|
|---|---|---|
|`reg_x`,`reg_y`,`reg_yaw`|0|Fix map↔world misalignment|
|`jump_threshold`|0 (off)|e.g. 0.3 → reject mocap flips|
|`publish_rate`|30 Hz|TF output rate|

> Option 2 (unused): fuse mocap + odom via `robot_localization` dual-EKF. Would need a relay (vrpn publishes `PoseStamped`, no covariance).

---

## Phase 4 — Nav2 Bring-up 

### The AMCL-free launch

`bringup_launch.py` = `localization_launch.py` (map_server + **AMCL**) + `navigation_launch.py`. We drop AMCL (it fights the mocap node over `map→odom`). **One-command bundle built:**

```bash
ros2 launch /root/maps/limo_mocap_nav2.launch.py
# starts map_server (auto-activated) + Nav2 (no AMCL) + mocap_map_odom
# args: reg_x reg_y reg_yaw jump_threshold nav_delay map params_file
```

Nav2 delayed 3 s so `/map` latches first.

### Troubleshooting

> [!bug] Map doesn't appear in RViz / "Robot out of bounds, no map received" `/map` is **Transient Local** (latched); RViz defaults to Volatile. **Fixes:** RViz Map display → Durability **Transient Local**. If costmap still complains: **restart RViz** (worked). Fallbacks: bounce `map_server` (deactivate→activate); check the map origin actually covers the robot's coords.

> [!bug] Lifecycle nodes — activate/deactivate `map_server` is a lifecycle node. `ros2 lifecycle get /map_server` → healthy = `active [3]`. States: `unconfigured --configure→ inactive --activate→ active`; `active --deactivate→ inactive`. **Bounce to re-publish latched `/map`:** `deactivate` then `activate`. **Gotcha:** `Unknown transition requested, available: deactivate, shutdown` just means it's **already active**. Not an error.

> [!bug] Orientation "jitter" in RViz — cosmetic only Goals reached fine. Orientation is noisier than position by nature. **Do NOT max Motive Smoothing** (adds control latency). Use `jump_threshold` for real jumps. **"Reached the goal fine" is the only test that matters.** RViz is a debug window, not the robot.

> [!bug] Robot stops short of goal — NOT an error **Goal tolerance** by design (Nav2 won't fine-tune). `xy_goal_tolerance: 0.15` → lower to **0.05** (mocap's accuracy supports it). Too tight → creeping/shuffling.

> [!bug] `Failed to populate message fields ... 'frame_id:'map''` YAML needs a **space after every colon**: `frame_id: 'map'`.

---

## Phase 5 — Goal Sending  (MATLAB deferred)

Goals go to **`/goal_pose`** (`geometry_msgs/PoseStamped`) or the **`/navigate_to_pose`** action. Since any ROS client can publish a `PoseStamped`, MATLAB adds nothing conceptually — **deferred as trivial**.

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}"
```

> [!bug] MATLAB couldn't send goals — two blockers
> 
> 1. **`Unrecognized message type nav2_msgs/NavigateThroughPosesFeedback`** — MATLAB doesn't ship `nav2_msgs`; would need `ros2genmsg`.
> 2. **`Subscriber did not receive any messages and timed out`** on `/vrpn_mocap/Limo/pose` — QoS mismatch (vrpn publishes **best-effort**, MATLAB subscribes **reliable**) and/or domain not set before MATLAB's ROS stack initialized.
> 
> **Resolution: send the route from the container CLI instead** (`send_route_action.m`, uses `navigate_to_pose`, which _blocks per goal_ so waypoints sequence automatically). MATLAB is kept for **offline bag analysis only** — that reads files, needs no network, and works fine.

---

## Phase 6 — Comparison Experiment 

**Design:** drive the **same waypoint route** under 3 localization regimes. OptiTrack records ground truth in **all** runs (it runs independently of what the robot navigates with).

|Run|Robot localizes with|Launch|Expect|
|---|---|---|---|
|`run1_mocap`|OptiTrack (`mocap_map_odom`)|the bundle|belief ≈ truth|
|`run2_amcl`|LiDAR↔map (AMCL)|`bringup_launch.py` + 2D Pose Estimate|belief ≈ truth, cm wobble|
|`run3_odom`|wheel odom only|static `map→odom` identity + `navigation_launch.py`|belief **drifts**|

**Route:** long L + loop back to start (straight legs → translational drift; corner → heading drift; loop closure → return error). Final waypoint yaw = **0** (same heading as origin) so loop closure measures position _and_ heading.

**Odom-only trick:** Nav2 needs a `map` frame, so publish a fake identity correction:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

### Per-run procedure

```bash
cd /root/maps                       # IMPORTANT: bags must land in the mounted folder
# 1. stack up (per run type)
# 2. record:
ros2 bag record -o run1_mocap /vrpn_mocap/Limo/pose /tf /tf_static /odom /plan
#    (wait ~2 s for the bag to warm up)
# 3. ./send_route_action.m                # blocks per waypoint; prints "Route complete"
# 4. Ctrl+C the bag
# 5. verify:  ros2 bag info run1_mocap
# 6. on the LAPTOP:  sudo chown -R zakaria:zakaria ~/maps/run1_mocap
```

Then `analyze_runs.m` in MATLAB (offline) → overlaid paths + drift plot + RMSE table.

### What we extract from each bag

- `/vrpn_mocap/Limo/pose` → **truth** (where the robot actually was)
- `/tf` → reconstruct `map→base_link` = **what the robot believed**
- `/odom` → raw wheel odometry · `/plan` → intended path

### run1_mocap — recorded ✅

145 s · 33,189 msgs · mocap 14,360 · tf 11,488 · odom 7,236 · plan 105. Healthy.

> `/tf_static` = 0 msgs. Harmless — it's latched and published once at bringup, before the bag started. Analysis reconstructs from dynamic `/tf`.

### Troubleshooting

> [!bug] `Output folder 'run1_mocap' already exists` `rm -rf run1_mocap` (if the attempt failed) or record to a new name. Check first with `ros2 bag info`.

> [!bug] MATLAB: "folder does not exist" / "Unable to read file ... .db3" **Root cause: container/host filesystem + user mismatch.**
> 
> - Bags recorded from `/` land at `/run1_mocap` — **invisible to the laptop.** Always `cd /root/maps` first.
> - Files written by the container are owned by **root**; MATLAB runs as `zakaria` and needs **write** access to the folder (it writes an index). World-readable isn't enough.
> - **Fix (on the laptop):** `sudo chown -R zakaria:zakaria ~/maps/run1_mocap`
> - If `metadata.yaml` is missing: `ros2 bag reindex run1_mocap`.

---

## 📦 Code / GitHub repo

Packaged as ROS 2 package **`mocap_localization`**:

```
mocap_localization/
├── README.md, LICENSE (MIT), CHANGELOG.md, .gitignore
├── package.xml, setup.py, setup.cfg
├── mocap_localization/mocap_map_odom.py
├── launch/{mocap_localization, limo_mocap_nav2}.launch.py
├── config/ (map + nav2.yaml), docker/Dockerfile
└── docs/{SETUP, DAILY, CONCEPTS}.md
```

Dockerfile bakes in the `netbase` fix + vrpn_mocap + Nav2. TODO before public push: replace `USERNAME` + name/email placeholders.

---

## 🔧 Open Issues

- [ ] **Registration re-check** — confirm the laser scan overlays the map walls in RViz. An earlier "out of bounds" at `(-0.21, -0.14)` while the robot _should_ have been well inside the map suggests a real `map`↔`world` offset. Fix with `reg_x/reg_y` or re-run Cartographer from the OptiTrack origin. **Resolve before trusting comparison data.**
- [ ] Record `run2_amcl` and `run3_odom`.
- [ ] Set real waypoint coords in `send_route_action.m` for the arena.

---

## 🧭 Conventions for this note

- Each **Phase**: _Goal · What we did · Key values · Decisions & why · Troubleshooting · Next_.
- Reusable values go in **Quick Reference**.
- Decisions record the **why**, not just the what.
- Bugs go in the phase's **Troubleshooting log** as symptom → cause → fix.

## References

- Mocap Nav2 — First Setup · Mocap Nav2 — Daily Manual · QuickRun Steps
- OptiTrack Motive · vrpn_mocap (alvinsunyixiao) · docs.nav2.org

---

_Part of the [LIMO documentation index](../README.md#documentation) · [repo home](../README.md)._
