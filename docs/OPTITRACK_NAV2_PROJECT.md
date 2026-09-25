# LIMO Pro + Nav2 + OptiTrack — absolute positioning (project log)

**Status:** Phases 1–5 complete · Phase 6 (comparison) in progress. OptiTrack →
ROS → mocap localizer (`map→odom`) → Nav2 → the robot drives waypoint routes.
Code lives in this repo as the `mocap_localization` package. Now collecting
comparison data (mocap vs AMCL vs odom-only).

**What this project is:** integrate an **AgileX LIMO Pro** (ROS 2 Foxy) with
**Nav2**, using **OptiTrack** motion capture as an **absolute** localization
source, on an existing **Cartographer** map (`mapMTR5`). The final phase compares
OptiTrack ground truth against the onboard estimates.

This is the deep-dive log: concepts, decisions and every bug met along the way.
For just running things, use [`OPTITRACK_NAV2_SETUP.md`](OPTITRACK_NAV2_SETUP.md)
(once) and [`OPTITRACK_NAV2_DAILY.md`](OPTITRACK_NAV2_DAILY.md) (each session).

---

## Quick reference

| Item | Value |
|---|---|
| Robot | AgileX LIMO Pro (Jetson Orin Nano, Ubuntu 20.04) |
| ROS distro | **Foxy** (EOL since May 2023) |
| Mocap driver | **`vrpn_mocap`** (runs on ARM; NatNet has no ARM build) |
| Motive PC IP (wired) | `192.168.8.184` (DHCP; verify) |
| LIMO IP (Wi-Fi) | `192.168.8.185` (DHCP; verify) |
| Rigid body name | **`Limo`** (capital L, no spaces) · Streaming ID `5` · 4 markers |
| VRPN port | `3883` · Motive Up Axis = **Z Up** |
| Mocap topic | **`/vrpn_mocap/Limo/pose`** (`PoseStamped`, frame `world`, 100 Hz) |
| **DDS / domain** | **`rmw_fastrtps_cpp`**, `ROS_DOMAIN_ID=10` (both machines) |
| Container | `limo_laptop` (`osrf/ros:foxy-desktop`), `--net=host`, mounts `~/ros2_ws` `~/maps` |
| **Moving base frame** | **`base_link`** (parent `odom`, 50 Hz); there is *no* `base_footprint` |
| Laser frame | `laser_link` (+0.180 m x, static) |
| Steering / controller | **Differential / DWB** |
| Map files | `/root/maps/mapMTR5.{yaml,pgm}` · `/root/maps/nav2.yaml` |
| One-command launch | `ros2 launch mocap_localization limo_mocap_nav2.launch.py` |
| Verify localization | `ros2 run tf2_ros tf2_echo map base_link` |

`~/maps` (laptop) and `/root/maps` (container) are the **same folder**. Anything
else in the container is invisible to the laptop.

---

## The six phases

1. **OptiTrack streaming:** Motive tracks the LIMO and streams over VRPN; ROS
   receives it as a topic.
2. **Coordinate frames (TF):** align the mocap pose with the map; build
   `map → odom → base_link`.
3. **Localization source:** replace AMCL's LiDAR guess with OptiTrack's absolute pose.
4. **Nav2 bring-up:** the navigation stack on the existing map; drive to a goal.
5. **Goal sending:** command "go to X" programmatically.
6. **Comparison:** mocap vs AMCL vs odom-only, against OptiTrack ground truth.

Two facts shaped the whole plan:

- **The LIMO's CPU is ARM.** OptiTrack's NatNet SDK has **no ARM build**, so we use
  **`vrpn_mocap`** (VRPN-only, open source).
- **Foxy is EOL.** Most docs target Humble/Jazzy; expect to back-port.

---

## Concept primer — `map → odom → base_link`

**Terminology.** A **frame** is a point of view (position + facing). A
**transform** is the offset converting between frames (the "→"). **TF** is ROS's
frame bookkeeping.

- **`base_link` = the robot itself.** Sensors are fixed offsets from it.
- **`map` = the room.** Fixed; goals and walls live here.
- **`odom` = a smooth, wheel-based tracker that drifts.** 50 Hz, never jumps,
  slowly wrong.

**Why two hops:** Nav2 wants `map → base_link` (the robot in the room). No single
sensor gives it directly, so:

```
map → base_link  =  map → odom   +   odom → base_link
(true position)     (correction)     (drifted odometry)
```

- `odom → base_link` = odometry (the robot publishes it).
- `map → odom` = **the correction**. Our node publishes it 30 times a second:
  `map→odom = (map→base) ∘ (odom→base)⁻¹`

**Who corrects whom:** mocap (truth) corrects odom (drifts), not the reverse.

**Why keep odom if mocap is absolute?**

1. **Smoothness:** it interpolates between discrete mocap frames.
2. **Dropout survival:** when mocap drops out in an arena dead spot, the last
   correction holds and odom carries the robot through.
3. **Convention:** REP-105 expects a continuous `odom → base_link`.

**Mental model: GPS + step counter.** Mocap = GPS (absolute, discrete, drops out
in tunnels). Odom = step counter (smooth, continuous, drifts). `map→odom` = the
GPS correction applied to the step-counter estimate.

**What AMCL was:** the default localizer. It *guesses* the position by matching
LiDAR to the map walls (a particle filter), needs a manual 2D Pose Estimate, and
is accurate to a few cm. Our node *knows* the pose instead. Both publish the
identical `map→odom`, so Nav2 can't tell the difference; that's why the swap is
clean.

---

## Phase 1 — OptiTrack streaming → ROS topic

**Result:** `/vrpn_mocap/Limo/pose` live at ~100 Hz, tracking the robot.

### Key rigid-body settings

| Setting | Value | Note |
|---|---|---|
| Min Marker Count | **3** | Survives 1 occluded marker (of 4). |
| Max Deflection | **~5–20 mm** | A tolerance, *not* noise control. 0 caused dropouts. |
| Smoothing | **0** (1–2 if jittery) | Adds latency; keep it low. |
| Forward Prediction | **0** | 200 caused flips at speed / sudden stops. |

### Decisions and why

- **`vrpn_mocap`:** the only driver that runs natively on ARM.
- **Up Axis = Z Up:** ROS is Z-up (REP-103); Motive defaults to Y-up.
- **Forward Prediction → 0:** extrapolation overshoots on a slow robot.
- **Uniform camera exposure/threshold:** mismatched settings make triangulation
  disagree.

### Troubleshooting

- **Flipping vs dropping.** *Flipping* = the marker pattern is too symmetric →
  use asymmetric, varied-height markers. (Worse than dropping: it sends a
  *confidently wrong* heading.) *Dropping* = markers lost → spread them out, mount
  them high, ≥2–3 cm apart. *Flips at speed / sudden stops* = Forward Prediction
  200 → set it to 0. *Residual flips in arena dead spots* = camera coverage gaps;
  accepted, handled with `jump_threshold`.
- **`getprotobyname() failed` / "VRPN connection is bad".** The container lacks
  `/etc/protocols`. Fix: `apt install -y netbase`. Recurs on every fresh container.
  (A `vrpn ver 07.35 vs 07.33` warning is harmless.)

---

## Phase 2 — Coordinate frames / TF

**Result:** `map → odom (30 Hz) → base_link (50 Hz) → {laser, imu, camera}`.
`tf2_echo map base_link` matches the true position.

### Registration

Assumed **`map ≡ world`** (identity). Evidence: at the origin, mocap read
`x=0.018, y=0.054`, yaw ≈ 1.2°.

- The node exposes `reg_x/reg_y/reg_yaw` (launch arguments) to correct a
  misalignment **without rebuilding the map**.
- Backup: re-run Cartographer starting at the OptiTrack origin, which makes
  `map ≡ world` by construction.

### Measured odom drift (why mocap matters)

Drove 134 cm → odom reported **127 cm**, plus ~7.5° heading drift on a "straight"
run.

### Foxy tooling gotchas

- `ros2 topic echo --once` / `--field` are **not supported in Foxy**. Echo, then
  `Ctrl+C` (or pipe through `grep`/`head`).
- `view_frames` needs the `.py`: `ros2 run tf2_tools view_frames.py`.
- No `base_footprint` exists; target `base_link`.

---

## Phase 3 — Mocap as the localization source

Custom package **`mocap_localization`**, node **`mocap_map_odom`**, running in
the container. It subscribes to `/vrpn_mocap/Limo/pose` (best-effort) and
publishes the `map→odom` TF at 30 Hz. **AMCL is not launched.** Source:
[`src/mocap_localization/mocap_localization/mocap_map_odom.py`](../src/mocap_localization/mocap_localization/mocap_map_odom.py).

| Param | Default | Use |
|---|---|---|
| `reg_x`, `reg_y`, `reg_yaw` | 0 | Fix map↔world misalignment |
| `jump_threshold` | 0 (off) | e.g. 0.3 → reject mocap flips |
| `jump_accept_frames` | 10 | Accept a real jump once it holds this many frames (so the filter can't lock out after a long dropout) |
| `publish_rate` | 30 Hz | TF output rate |

Option 2 (unused): fuse mocap + odom with a `robot_localization` dual EKF. That
would need a relay node (vrpn publishes `PoseStamped`, with no covariance).

---

## Phase 4 — Nav2 bring-up

### The AMCL-free launch

`bringup_launch.py` = `localization_launch.py` (map_server + **AMCL**) +
`navigation_launch.py`. We drop AMCL (it fights the mocap node over `map→odom`).
**One-command bundle:**

```bash
ros2 launch mocap_localization limo_mocap_nav2.launch.py
# starts map_server (auto-activated) + Nav2 (no AMCL) + mocap_map_odom
# args: map params_file reg_x reg_y reg_yaw jump_threshold use_sim_time
```

Nav2 is delayed 3 s so `/map` latches first. (An identical copy lives at
`config/limo_mocap_nav2.launch.py` → `~/maps/`, runnable as
`ros2 launch /root/maps/limo_mocap_nav2.launch.py`.)

### Troubleshooting

- **Map doesn't appear in RViz / "Robot out of bounds, no map received".** `/map`
  is **Transient Local** (latched); RViz defaults to Volatile. Fix: RViz Map
  display → Durability **Transient Local**. If the costmap still complains,
  **restart RViz** (that worked). Fallbacks: bounce `map_server`
  (deactivate → activate); check that the map origin actually covers the robot's
  coordinates.
- **Lifecycle nodes.** `map_server` is a lifecycle node.
  `ros2 lifecycle get /map_server` → healthy = `active [3]`. States:
  `unconfigured --configure→ inactive --activate→ active`;
  `active --deactivate→ inactive`. **Bounce it to re-publish the latched `/map`:**
  `deactivate`, then `activate`. `Unknown transition requested, available:
  deactivate, shutdown` just means it's **already active**; not an error.
- **Orientation "jitter" in RViz is cosmetic.** Goals were reached fine.
  Orientation is noisier than position by nature. **Do NOT max out Motive
  Smoothing** (it adds control latency). Use `jump_threshold` for real jumps.
  "Reached the goal" is the test that matters; RViz is a debug window.
- **Robot stops short of the goal: not an error.** That's the **goal tolerance**,
  by design. `xy_goal_tolerance` in `nav2.yaml` (0.05 in the repo; mocap's
  accuracy supports it). Too tight → creeping/shuffling at the goal.
- **`Failed to populate message fields ... 'frame_id:'map''`.** YAML needs a
  **space after every colon**: `frame_id: 'map'`.

---

## Phase 5 — Goal sending

Goals go to **`/goal_pose`** (`geometry_msgs/PoseStamped`) or to the
**`/navigate_to_pose`** / **`/navigate_through_poses`** actions.

From the CLI:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}"
```

From MATLAB (`matlab/goals/`, see
[CONTROL_SETUP](CONTROL_SETUP.md#1-put-the-matlab-folders-on-the-path) for the path):

| Script | Needs | What it does |
|---|---|---|
| `limo_connect.m` | nothing extra | node + publishers/subscribers with the correct QoS and domain |
| `limo_goal.m` | nothing extra | one goal via `/goal_pose`; `wait=true` blocks until mocap says it arrived |
| `send_route_goalpose.m` | nothing extra | the fixed comparison route, goal by goal (**recommended**) |
| `send_route_action.m` | `gen_nav2_msgs.m` run once | the same route via the `navigate_through_poses` action, with a result |
| `limo_state.m` | nothing extra | prints truth / Nav2 estimate / odom / battery / drift |

**Two blockers we hit early, and their fixes** (both now built into `limo_connect`):

1. **`Unrecognized message type nav2_msgs/NavigateThroughPosesFeedback`.** MATLAB
   doesn't ship `nav2_msgs`. Either generate it once with
   [`matlab/setup/gen_nav2_msgs.m`](../matlab/setup/gen_nav2_msgs.m) (`ros2genmsg`),
   or avoid actions and use `/goal_pose` (`send_route_goalpose.m`).
2. **`Subscriber did not receive any messages and timed out`** on
   `/vrpn_mocap/Limo/pose`. Two causes: a QoS mismatch (MATLAB subscribes
   **reliable** by default; a best-effort subscriber matches any publisher, so
   `limo_connect` always uses best-effort), and/or the domain not set before
   MATLAB's ROS stack initialized (`limo_connect` passes `domainID` to `ros2node`
   directly). If MATLAB sees robot topics but not the container's, it's the Fast
   DDS shared-memory problem: see
   [CONTROL_SETUP §3](CONTROL_SETUP.md#3-the-dds-fix-one-time-file-used-every-session).

Offline bag analysis in MATLAB reads files only (no network), so it was never
affected by either.

---

## Phase 6 — Comparison experiment

**Design:** drive the **same waypoint route** under 3 localization regimes.
OptiTrack records ground truth in **all** runs (it runs independently of what the
robot navigates with).

| Run | Robot localizes with | Launch | Expect |
|---|---|---|---|
| `run1_mocap` | OptiTrack (`mocap_map_odom`) | the bundle | belief ≈ truth |
| `run2_amcl` | LiDAR↔map (AMCL) | `bringup_launch.py` + 2D Pose Estimate | belief ≈ truth, cm wobble |
| `run3_odom` | wheel odom only | static `map→odom` identity + map_server + `navigation_launch.py` | belief **drifts** |

For `run2_amcl` and `run3_odom`, **Terminal B (`vrpn_mocap`) must still run** so
the truth gets recorded, but `mocap_map_odom` must **not** run (it would correct
the drift you're trying to measure).

**Route:** a long L + loop back to the start (straight legs → translational drift;
corner → heading drift; loop closure → return error). The final waypoint yaw is
**0** (same heading as the origin), so loop closure measures position *and*
heading. Start every run with the robot at (0, 0) facing +x.

**Odom-only trick:** Nav2 needs a `map` frame, so publish a fake identity
correction:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

### Per-run procedure

In a container shell:

```bash
cd /root/maps                       # IMPORTANT: bags must land in the mounted folder
# 1. bring the stack up for this run type (table above)
# 2. record:
ros2 bag record -o run1_mocap /vrpn_mocap/Limo/pose /tf /tf_static /odom /plan
#    (wait ~2 s for the recorder to subscribe)
```

In MATLAB:

```matlab
send_route_goalpose        % blocks per waypoint; prints "Route complete"
```

Back in the container and on the laptop:

```bash
# 3. Ctrl+C the bag recorder
# 4. verify (container):  ros2 bag info run1_mocap
# 5. fix ownership (LAPTOP host):  sudo chown -R $USER:$USER ~/maps/run1_mocap
# 6. file it in the repo:           cp -r ~/maps/run1_mocap ~/limo/data/
```

Then run [`analyze_runs.m`](../matlab/analysis/analyze_runs.m) in MATLAB (offline).
It reads the bags from `data/`, and produces overlaid paths, a drift plot and an
RMSE table. Add `run2_amcl` / `run3_odom` to its `runs` struct as you record them.

### What we extract from each bag

- `/vrpn_mocap/Limo/pose` → **truth** (where the robot actually was)
- `/tf` → reconstruct `map→base_link` = **what the robot believed**
- `/odom` → raw wheel odometry · `/plan` → the intended path

### run1_mocap — recorded

145 s · 33,189 msgs · mocap 14,360 · tf 11,488 · odom 7,236 · plan 105. Healthy.
Shipped in [`data/run1_mocap/`](../data/run1_mocap/).

`/tf_static` = 0 msgs. Harmless: it's latched and was published once at
bring-up, before the bag started. The analysis reconstructs from the dynamic `/tf`.

### Troubleshooting

- **`Output folder 'run1_mocap' already exists`.** `rm -rf run1_mocap` (if the
  attempt failed) or record under a new name. Check first with `ros2 bag info`.
- **MATLAB: "folder does not exist" / "Unable to read file ... .db3".** The root
  cause is a container/host filesystem and user mismatch:
  - Bags recorded from `/` land at `/run1_mocap`, **invisible to the laptop**.
    Always `cd /root/maps` first.
  - Files written by the container are owned by **root**. MATLAB runs as your user
    and needs **write** access to the folder (it writes an index); world-readable
    isn't enough.
  - **Fix (on the laptop):** `sudo chown -R $USER:$USER ~/maps/run1_mocap`
  - If `metadata.yaml` is missing: `ros2 bag reindex run1_mocap`.
- **Truth and belief look offset in time.** Mocap stamps come from the laptop's
  clock; `odom→base_link` stamps come from the robot's. Keep the clocks synced
  (`chrony`/NTP).

---

## Code in this repo

| Path | What |
|---|---|
| `src/mocap_localization/mocap_localization/mocap_map_odom.py` | the localizer node |
| `src/mocap_localization/launch/mocap_localization.launch.py` | localizer only (daily Terminal C) |
| `src/mocap_localization/launch/limo_mocap_nav2.launch.py` | one-command bundle (map_server + Nav2 without AMCL + localizer) |
| `src/mocap_localization/install_mocap_localization.sh` | apt deps + build, inside the container |
| `src/mocap_localization/docker/Dockerfile` | Foxy image with `netbase`, `vrpn_mocap`, Nav2 baked in (unverified) |
| `matlab/goals/`, `matlab/analysis/` | goal sending and bag analysis (Phases 5–6) |
| `data/run1_mocap/` | the recorded mocap run |

---

## Open issues

- [ ] **Registration re-check:** confirm the laser scan overlays the map walls in
  RViz. An earlier "out of bounds" at `(-0.21, -0.14)`, while the robot *should*
  have been well inside the map, suggests a real `map`↔`world` offset. Fix with
  `reg_x/reg_y`, or re-run Cartographer from the OptiTrack origin. **Resolve this
  before trusting the comparison data.**
- [ ] Record `run2_amcl` and `run3_odom`.
- [ ] Set real waypoint coordinates for the arena in `send_route_goalpose.m`
  (and `send_route_action.m`, which uses the same route).
- [ ] Verify the Dockerfile builds and works, then remove its "reconstructed" banner.

---

## Conventions for this log

- Each **phase**: *Goal · What we did · Key values · Decisions & why ·
  Troubleshooting · Next*.
- Reusable values go in **Quick reference**.
- Decisions record the **why**, not just the what.
- Bugs go in the phase's **Troubleshooting** list as symptom → cause → fix.

## References

- [Mocap first-time setup](OPTITRACK_NAV2_SETUP.md) · [Mocap daily manual](OPTITRACK_NAV2_DAILY.md)
- OptiTrack Motive documentation: <https://docs.optitrack.com>
- `vrpn_mocap` (alvinsunyixiao): <https://github.com/alvinsunyixiao/vrpn_mocap>
- Nav2: <https://docs.nav2.org>

---

_Part of the [LIMO documentation index](../README.md#documentation) · [repo home](../README.md)._
