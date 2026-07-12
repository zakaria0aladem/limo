# LIMO Tutorial — from reactive motion to absolute-localized control

This repo documents everything built on the **AgileX LIMO Pro**, arranged as a
progression. Each level is self-contained and runnable; later levels assume the
setup from earlier ones (robot drivers, the Foxy container, domain 10, Fast DDS).

Work top to bottom, or jump straight to the level you need.

---

## Level 0 — Reactive navigation (no map, no localization)

The simplest possible autonomy: react to the laser scan, no map at all. A
potential-field node builds a repulsion vector from `/scan`, adds a constant
attraction vector, and drives `/cmd_vel`. Good for confirming the robot moves
and the scan is sane before anything harder.

- **Code:** `src/limo_nav/` (node: `wandering`)
- **Guide:** [WANDERING.md](WANDERING.md)
- **Run:** `ros2 run limo_nav wandering`

## Level 1 — Mapping & navigation with the onboard LiDAR (AMCL + SLAM)

The standard ROS 2 stack. Two parts:

1. **SLAM** (`slam_toolbox`) — drive the robot to *build* a map, with live loop
   closure. Save it for reuse.
2. **AMCL navigation** — load a saved map, localize by matching LiDAR to walls
   (a particle filter you seed with **2D Pose Estimate**), then send goals.

This is the baseline the mocap work later *replaces the localizer of*.

- **Config:** `config/nav2.yaml`, `config/slam_params.yaml`, `config/mapMTR5.{yaml,pgm}`
- **Guides:** [SLAM.md](SLAM.md) (build/save a map with slam_toolbox) then
  [NAVIGATION.md](NAVIGATION.md) (AMCL navigation on that map + Nav2 speed tuning).
- **First?** Do the shared [DEVICE_SETUP.md](DEVICE_SETUP.md) once before any level.
- **Key LIMO quirk:** the base frame is `base_link`, not `base_footprint` —
  the SLAM config and the `-p base_frame:=base_link` override both handle it.

## Level 2 — Absolute localization with OptiTrack (mocap replaces AMCL)

The heart of the project. A custom node turns the OptiTrack pose into the
`map → odom` transform — the exact contract AMCL fulfils — so the robot knows
its true, drift-free position with **no 2D Pose Estimate and no drift**. Nav2 is
otherwise unchanged.

```
OptiTrack (Motive) ──VRPN──▶ vrpn_mocap ──▶ mocap_map_odom ──map→odom──▶ Nav2
```

- **Code:** `src/mocap_localization/` (node `mocap_map_odom`, plus the
  one-command `limo_mocap_nav2.launch.py` bundle)
- **Guides:** [OPTITRACK_NAV2_SETUP.md](OPTITRACK_NAV2_SETUP.md) (one-time:
  Motive rigid body, VRPN, registration), [OPTITRACK_NAV2_DAILY.md](OPTITRACK_NAV2_DAILY.md)
  (per-session run), [OPTITRACK_NAV2_PROJECT.md](OPTITRACK_NAV2_PROJECT.md)
  (the full phase-by-phase log, the `map→odom→base_link` concept primer, and the
  mocap-vs-AMCL-vs-odom **comparison experiment**).
- **Analysis:** `matlab/analysis/analyze_runs.m` reads the recorded bag in
  `data/run1_mocap/` and plots believed path vs OptiTrack truth.

## Level 3 — Closed-loop control from MATLAB/Simulink (Nav2 bypassed)

With a drift-free state estimate, the LIMO becomes a clean control testbed. A
Simulink model closes the loop directly on the mocap pose and drives `/cmd_vel`,
with a **Variant Subsystem** that swaps P / PID / LQR (and later MPC / DeePC /
Koopman) behind one fixed interface `state[3], goal[3] → u=[v; w]`. Nav2 is off.

- **Code:** `matlab/control/` (`limo_ctrl_params.m` designs gains incl. a
  toolbox-free LQR; `build_limo_control_model.m` generates the `.slx`),
  `matlab/goals/` (connect / read state / send goals & routes)
- **Guides:** [CONTROL_SETUP.md](CONTROL_SETUP.md) (one-time),
  [CONTROL_DAILY.md](CONTROL_DAILY.md) (per-session run + safety)
- **Safety:** two E-STOP manual switches start in the zero position; motion
  only on a deliberate double-click. First runs at `v_max = 0.15 m/s`.

---

## The one cross-cutting gotcha: Fast DDS

Any time a host process (MATLAB, or a second container) must see topics coming
from *inside* the container, Fast DDS's shared-memory transport silently drops
data across the container/host boundary. The fix is UDP-only, via
`config/fastdds_udp.xml` — read its header, and apply it on **both** sides.
This shows up in Levels 2 and 3.

## Environment (all levels)

- ROS 2 **Foxy** on the LIMO; a `osrf/ros:foxy-desktop` container on the laptop
  (`--net=host`, mounts `~/ros2_ws` and `~/maps`)
- `rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=10` on **both** machines
- MATLAB **R2023a** for Levels 2 (analysis) and 3 (control)

Build the ROS packages once: `colcon build --symlink-install` from the repo root,
then `source install/setup.bash`.

---

_Part of the [LIMO documentation index](../README.md#documentation) · [repo home](../README.md)._
