# LIMO Pro — OptiTrack · Nav2 · MATLAB/Simulink

A consolidated, tutorial-style workspace for the **AgileX LIMO Pro**: mapping and
autonomous navigation with **Nav2**, **absolute localization** from an
**OptiTrack** motion-capture system, and **closed-loop control** from
**MATLAB/Simulink** — all on one robot, documented end to end.

The core idea: a small node (`mocap_map_odom`) turns the OptiTrack pose into the
`map → odom` transform, **replacing AMCL** as Nav2's global localizer. Both
publish the identical transform, so the swap is invisible to the rest of Nav2 —
but the robot now knows its true, drift-free position with no manual pose init.

```
OptiTrack (Motive) ──VRPN──▶ vrpn_mocap ──▶ mocap_map_odom ──map→odom──▶ Nav2 ──▶ /cmd_vel ──▶ LIMO
```

---

## Table of contents

- [Start here](#start-here)
- [Documentation](#documentation)
- [The four workflows](#the-four-workflows)
- [Repository layout](#repository-layout)
- [Quickstart](#quickstart)
- [Conventions and key facts](#conventions-and-key-facts)
- [Troubleshooting index](#troubleshooting-index)
- [Requirements](#requirements)
- [Hardware](#hardware)
- [Status and reconstructed files](#status-and-reconstructed-files)
- [License](#license)

---

## Start here

New to the repo? Read the guided path first:

**[`docs/TUTORIAL.md`](docs/TUTORIAL.md)** — a progression from reactive motion
→ SLAM/AMCL → mocap absolute localization → closed-loop control.

Then do the one-time base setup every workflow depends on:

**[`docs/DEVICE_SETUP.md`](docs/DEVICE_SETUP.md)** — robot bring-up (incl. the
vendor LiDAR), network, laptop container, `limo_msgs`, Nav2 install.

---

## Documentation

Every doc, and what it's for:

| Doc | Covers |
|---|---|
| [TUTORIAL.md](docs/TUTORIAL.md) | The learning path tying all workflows together |
| [DEVICE_SETUP.md](docs/DEVICE_SETUP.md) | **One-time base setup** — robot, LiDAR bring-up, container, Nav2, `limo_msgs` |
| [WANDERING.md](docs/WANDERING.md) | **Level 0** — reactive potential-field demo (`limo_nav`), no map |
| [SLAM.md](docs/SLAM.md) | **Level 1a** — build a map with `slam_toolbox` (loop closure, save/continue) |
| [NAVIGATION.md](docs/NAVIGATION.md) | **Level 1b** — AMCL navigation on a saved map + Nav2 speed tuning |
| [OPTITRACK_NAV2_SETUP.md](docs/OPTITRACK_NAV2_SETUP.md) | **Level 2** one-time — Motive rigid body, VRPN, registration |
| [OPTITRACK_NAV2_DAILY.md](docs/OPTITRACK_NAV2_DAILY.md) | **Level 2** per-session run |
| [OPTITRACK_NAV2_PROJECT.md](docs/OPTITRACK_NAV2_PROJECT.md) | **Level 2** deep dive — full phase log, `map→odom` primer, comparison experiment |
| [CONTROL_SETUP.md](docs/CONTROL_SETUP.md) | **Level 3** one-time — Simulink control testbed setup |
| [CONTROL_DAILY.md](docs/CONTROL_DAILY.md) | **Level 3** per-session run + safety (E-STOP) |
| [matlab/perception/README.md](matlab/perception/README.md) | Side track — LiDAR obstacle detection in MATLAB |

---

## The four workflows

Only **one** node may publish `/cmd_vel` at a time — run one workflow, not two.

| Level | Workflow | What drives the robot | Localization | Guide |
|---|---|---|---|---|
| 0 | Reactive wandering | potential field on `/scan` | none | [WANDERING.md](docs/WANDERING.md) |
| 1 | SLAM + AMCL nav | Nav2 planner + controller | LiDAR ↔ map | [SLAM.md](docs/SLAM.md), [NAVIGATION.md](docs/NAVIGATION.md) |
| 2 | Mocap Nav2 | Nav2 planner + controller | OptiTrack (`mocap_map_odom`) | [OPTITRACK_NAV2_DAILY.md](docs/OPTITRACK_NAV2_DAILY.md) |
| 3 | Closed-loop control | Simulink P/PID/LQR, Nav2 bypassed | OptiTrack | [CONTROL_DAILY.md](docs/CONTROL_DAILY.md) |

---

## Repository layout

```
limo/
├── README.md                     you are here
├── docs/                         all setup + daily manuals (see Documentation)
├── src/                          ROS 2 packages (colcon)
│   ├── limo_msgs/                LimoStatus.msg interface
│   ├── limo_nav/                 reactive potential-field "wandering" node
│   └── mocap_localization/       OptiTrack map→odom localizer + Nav2 bringup
│       ├── mocap_localization/mocap_map_odom.py
│       ├── launch/               mocap_localization + limo_mocap_nav2 (one-command)
│       ├── install_mocap_localization.sh
│       └── docker/Dockerfile
├── matlab/
│   ├── control/                  build_limo_control_model.m, limo_ctrl_params.m
│   ├── goals/                    limo_connect, limo_state, limo_goal, send_route_*
│   ├── perception/               LiDAR obstacle detection + live viewer (/scan)
│   ├── analysis/                 analyze_runs.m (truth vs belief, RMSE/drift)
│   ├── setup/                    gen_nav2_msgs.m (one-time action interface)
│   └── examples/                 ROS 2 scratch snippets
├── config/                       nav2.yaml, slam_params.yaml, map, fastdds_udp.xml,
│                                 limo_mocap_nav2.launch.py (copy for ~/maps),
│                                 optitrack/ (Motive project + calibration)
└── data/run1_mocap/              example rosbag (so analyze_runs runs on clone)
```

---

## Quickstart

**0. One-time:** follow [DEVICE_SETUP.md](docs/DEVICE_SETUP.md). In short, on the
laptop host:

```bash
git clone https://github.com/zakaria0aladem/limo.git ~/limo
mkdir -p ~/ros2_ws/src ~/maps
cp -r ~/limo/src/mocap_localization ~/limo/src/limo_nav ~/ros2_ws/src/
cp ~/limo/config/{mapMTR5.yaml,mapMTR5.pgm,nav2.yaml,slam_params.yaml,fastdds_udp.xml,limo_mocap_nav2.launch.py} ~/maps/
```

then, inside the Foxy container (`~/ros2_ws` is mounted at `/root/ros2_ws`):

```bash
cd /root/ros2_ws
colcon build --packages-select limo_msgs mocap_localization limo_nav
source install/setup.bash
```

`limo_msgs` normally comes from AgileX's `limo_ros2` already in `~/ros2_ws/src`;
don't add a second copy (DEVICE_SETUP step 5).

**Mocap + Nav2** (the flagship workflow):

```bash
# robot (SSH):        ros2 launch limo_bringup limo_start.launch.py
# mocap (container):  ros2 launch vrpn_mocap client.launch.yaml server:=<MOTIVE_PC_IP> port:=3883
# laptop stack:       ros2 launch mocap_localization limo_mocap_nav2.launch.py
```

Then send a goal from RViz (2D Goal Pose) or CLI. Per-workflow step-by-step,
troubleshooting, and Motive configuration are in [docs/](docs); the full
terminal-by-terminal run is [OPTITRACK_NAV2_DAILY.md](docs/OPTITRACK_NAV2_DAILY.md).

---

## Conventions and key facts

- **DDS / domain:** `rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=10` on **both** machines.
- **Moving base frame:** `base_link` (this LIMO has **no** `base_footprint`).
- **Mocap topic:** `/vrpn_mocap/Limo/pose` (`PoseStamped`, best-effort, ~100 Hz).
- **`fastdds_udp.xml` is required** whenever a host process (MATLAB, a second
  container) must see topics from *inside* the container — it forces UDP-only so
  the container/host shared-memory boundary doesn't silently drop data. See the
  header of [`config/fastdds_udp.xml`](config/fastdds_udp.xml).
- **Foxy is EOL** — most upstream docs target Humble/Jazzy; expect back-porting.
  Foxy's `ros2 topic echo` has no `--once` / `--field`.
- **Only one `/cmd_vel` publisher at a time** — wandering, Nav2, or Simulink.
- **Save everything under `~/maps`** (`/root/maps` in the container): it is the
  only folder shared by the container, the host, and MATLAB, and it survives
  container rebuilds.

---

## Troubleshooting index

The most common problems, and where the fix is documented:

| Symptom | Usual cause | Fix |
|---|---|---|
| Container sees no robot topics | domain / RMW mismatch, subnet, firewall | [DEVICE_SETUP checklist](docs/DEVICE_SETUP.md#checklist-is-the-base-setup-done) |
| `ros2: command not found` | base ROS not sourced | [DEVICE_SETUP step 5](docs/DEVICE_SETUP.md#5-build-limo_msgs-in-the-container-once) |
| `Deserialization of data failed` | `limo_msgs` not built/sourced | [DEVICE_SETUP step 5](docs/DEVICE_SETUP.md#5-build-limo_msgs-in-the-container-once) |
| `Duplicate package names not supported: limo_msgs` | two copies in `src/` | [DEVICE_SETUP step 5](docs/DEVICE_SETUP.md#5-build-limo_msgs-in-the-container-once) |
| `getprotobyname() failed` / VRPN "connection is bad" | `netbase` missing | [OPTITRACK_NAV2_SETUP Part 2](docs/OPTITRACK_NAV2_SETUP.md#part-2--install-the-vrpn-driver-container) |
| Map not visible in RViz | Durability not Transient Local / map_server inactive | [OPTITRACK_NAV2_DAILY](docs/OPTITRACK_NAV2_DAILY.md#map_server-lifecycle) |
| Robot pose jumps between two places | AMCL and `mocap_map_odom` both running | [OPTITRACK_NAV2_DAILY Terminal E](docs/OPTITRACK_NAV2_DAILY.md#terminal-e--nav2-navigation-no-amcl-container) |
| Laser scan doesn't overlay map walls | map ↔ world registration | [OPTITRACK_NAV2_SETUP Part 4](docs/OPTITRACK_NAV2_SETUP.md#part-4--registration-map--world-alignment) |
| MATLAB sees robot topics but not the container's | Fast DDS shared memory | [CONTROL_SETUP §3](docs/CONTROL_SETUP.md#3-the-dds-fix-one-time-file-used-every-session) |
| MATLAB `receive` times out on mocap | QoS / domain | use `limo_connect` ([project log, Phase 5](docs/OPTITRACK_NAV2_PROJECT.md#phase-5--goal-sending)) |
| MATLAB can't read a recorded bag | bag outside `~/maps` or root-owned | [project log, Phase 6](docs/OPTITRACK_NAV2_PROJECT.md#troubleshooting-2) |
| SLAM never produces a map | `base_footprint` vs `base_link` | [SLAM.md](docs/SLAM.md#troubleshooting) |

---

## Requirements

- ROS 2 **Foxy** on the LIMO; a `osrf/ros:foxy-desktop` container on the laptop
  (`--net=host`, mounts `~/ros2_ws` and `~/maps`)
- `ros-foxy-navigation2`, `ros-foxy-nav2-bringup`, `ros-foxy-slam-toolbox`,
  `ros-foxy-vrpn-mocap`, `netbase`
- OptiTrack Motive + a calibrated capture volume (Levels 2–3)
- MATLAB **R2023a** (last release shipping ROS 2 Foxy) + ROS Toolbox (analysis/control)

---

## Hardware

- **AgileX LIMO Pro** — Jetson Orin Nano, Ubuntu 20.04, ROS 2 Foxy, differential drive
- Onboard **YDLIDAR** + wheel odometry (AgileX vendor stack, on the robot)
- **OptiTrack** motion-capture system streaming over VRPN
- Windows PC running **Motive**; a laptop running Ubuntu + the Foxy container

---

## Status and reconstructed files

- The comparison experiment ([`analyze_runs.m`](matlab/analysis/analyze_runs.m))
  ships one recorded run (`data/run1_mocap/`); `run2_amcl` / `run3_odom` extend
  the `runs` struct when recorded.
- A few files are **reconstructed** (not the lost originals) and marked in-file —
  verify against your setup: [`matlab/goals/limo_state.m`](matlab/goals/limo_state.m),
  [`config/slam_params.yaml`](config/slam_params.yaml), and
  `src/mocap_localization/docker/Dockerfile`.
- `src/mocap_localization/install_mocap_localization.sh` was rewritten to copy
  (not symlink) the package into the workspace; not yet run on hardware.
- `config/nav2.yaml` keeps `map_server.yaml_filename: turtlebot3_world.yaml` (a
  Nav2 default). Every launch in these docs overrides it with the real map, so it
  only matters if you start `map_server` from that file alone.
- The MATLAB control model (`limo_mocap_control.slx`) is generated, not
  committed: run `build_limo_control_model` after cloning or after changing it.

---

## License

MIT — see [LICENSE](LICENSE).
