# LIMO Pro — OptiTrack · Nav2 · MATLAB/Simulink

A consolidated workspace for the **AgileX LIMO Pro** (Jetson Orin Nano, Ubuntu 20.04, ROS 2 **Foxy**): absolute localization from an **OptiTrack** motion-capture system, autonomous navigation with **Nav2**, and closed-loop control from **MATLAB/Simulink**.

The core idea: a small node (`mocap_map_odom`) turns the OptiTrack pose into the `map → odom` transform, **replacing AMCL** as Nav2's global localizer. Because both publish the identical transform, the swap is invisible to the rest of Nav2 — but the robot now knows its true, drift-free position with no manual pose initialization.

```
OptiTrack (Motive) ──VRPN──▶ vrpn_mocap ──▶ mocap_map_odom ──map→odom──▶ Nav2 ──▶ /cmd_vel ──▶ LIMO
```

## Two workflows

| Workflow | What drives the robot | Localization | Entry point |
|---|---|---|---|
| **Mocap + Nav2** | Nav2 planner + controller | OptiTrack (`mocap_map_odom`) | `docs/OPTITRACK_NAV2_DAILY.md` |
| **Closed-loop control** | Simulink model (P / PID / LQR) on `/cmd_vel`, Nav2 bypassed | OptiTrack | `docs/CONTROL_DAILY.md` |

Only **one** publisher may own `/cmd_vel` at a time — run one workflow or the other, never both.

## Layout

```
src/                         ROS 2 packages (colcon)
  limo_msgs/                 LimoStatus.msg interface
  limo_nav/                  reactive potential-field "wandering" demo node
  mocap_localization/        OptiTrack map→odom localizer + one-command Nav2 bringup
matlab/
  control/                   Simulink builder + params (toolbox-free LQR)
  goals/                     connect / state / goal / route senders
  analysis/                  offline rosbag comparison (truth vs belief)
  setup/                     one-time nav2_msgs generation
  examples/                  ROS 2 scratch snippets
config/                      nav2.yaml, map, fastdds_udp.xml
data/run1_mocap/             example rosbag (so analysis runs out of the box)
docs/                        setup + daily manuals and the full project log
```

## Quickstart (mocap + Nav2)

```bash
# 1) build the ROS 2 packages (in your Foxy environment / container)
cd <this-repo>
colcon build --symlink-install
source install/setup.bash

# 2) copy the runtime files into the mounted maps folder the launch files expect
cp config/{mapMTR5.yaml,mapMTR5.pgm,nav2.yaml,fastdds_udp.xml} ~/maps/

# 3) robot drivers (SSH to the LIMO)
ros2 launch limo_bringup limo_start.launch.py

# 4) mocap driver (container) — use YOUR Motive PC IP
ros2 launch vrpn_mocap client.launch.yaml server:=<MOTIVE_PC_IP> port:=3883

# 5) one-command laptop-side stack: map_server + Nav2 (no AMCL) + mocap_map_odom
ros2 launch mocap_localization limo_mocap_nav2.launch.py
```

Send a goal from RViz (2D Goal Pose) or the CLI. Full step-by-step, troubleshooting, and the OptiTrack/Motive configuration are in `docs/`.

## Key facts (this project's conventions)

- **DDS / domain:** `rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=10` on both machines.
- **Moving base frame:** `base_link` (no `base_footprint`).
- **Mocap topic:** `/vrpn_mocap/Limo/pose` (`PoseStamped`, best-effort, ~100 Hz).
- **The `fastdds_udp.xml` profile is required** whenever MATLAB or another host process must see topics from inside the container — it forces UDP-only so the container/host shared-memory boundary doesn't silently drop data. See the header of `config/fastdds_udp.xml`.
- **Foxy is EOL** — most upstream docs target Humble/Jazzy; expect occasional back-porting.

## Requirements

- ROS 2 Foxy (LIMO) + a Foxy desktop container on the laptop
- Nav2 (`ros-foxy-navigation2`, `ros-foxy-nav2-bringup`), `ros-foxy-vrpn-mocap`, `netbase`
- OptiTrack Motive + a calibrated capture volume
- MATLAB **R2023a** (last release shipping ROS 2 Foxy) with ROS Toolbox, for the control/analysis paths

## Status & notes

- The comparison experiment (`matlab/analysis/analyze_runs.m`) currently ships one recorded run (`run1_mocap`); `run2_amcl` / `run3_odom` extend the `runs` struct when recorded.
- A few files are **reconstructed** (not the lost originals) and marked in-file: `matlab/goals/limo_state.m`, `src/mocap_localization/install_mocap_localization.sh`, and `src/mocap_localization/docker/Dockerfile`. Verify these against your working setup.

## License

MIT — see `LICENSE`.
