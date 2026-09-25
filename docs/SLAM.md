# SLAM — build a map with the onboard LiDAR (slam_toolbox)

Online SLAM with `slam_toolbox`: drive the robot to *build* a map while it
navigates, with live loop closure. Save the result and reuse it for AMCL
navigation ([`NAVIGATION.md`](NAVIGATION.md)).

slam_toolbox runs alongside Nav2's navigation stack, so the robot maps an area
**and** navigates at the same time. slam_toolbox publishes both `/map` and the
`map → odom` transform, so no AMCL, no map_server, and no 2D Pose Estimate.

**Prerequisite:** the shared base setup in [`DEVICE_SETUP.md`](DEVICE_SETUP.md)
(robot bring-up, container, Nav2, `~/maps` populated).

---

## One-time setup (in the container)

Install slam_toolbox:

```bash
apt update
apt install -y ros-foxy-slam-toolbox
```

The mapping parameters are committed at
[`config/slam_params.yaml`](../config/slam_params.yaml) and copied to `~/maps`
in DEVICE_SETUP step 6. Check it is there (it lives on the mounted folder, so
it survives container rebuilds):

```bash
ls /root/maps/slam_params.yaml
grep base_frame /root/maps/slam_params.yaml     # must show base_link
```

**Why `base_link`:** the slam_toolbox default config uses `base_footprint`, but
the LIMO has no `base_footprint` frame. With the default, slam_toolbox waits
forever for a transform that never comes and no map appears. The repo file is
the stock `mapper_params_online_async.yaml` with that one edit. To regenerate it
from the stock file:

```bash
cp /opt/ros/foxy/share/slam_toolbox/config/mapper_params_online_async.yaml /root/maps/slam_params.yaml
sed -i 's/base_footprint/base_link/g' /root/maps/slam_params.yaml
```

Do not keep configs anywhere else in the container (for example
`/root/limo_workspace/`): only `/root/maps` and `/root/ros2_ws` are mounted, and
everything else is lost when the container is recreated.

---

## Daily SLAM workflow (4 terminals)

### Terminal A — Robot drivers (SSH)

```bash
ssh agilex@192.168.8.185        # verify the IP with hostname -I on the LIMO
```

```bash
pkill -9 ros2
unset FASTRTPS_DEFAULT_PROFILES_FILE
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
ros2 launch limo_bringup limo_start.launch.py
```

Wait for `Now lidar is scanning...`. Leave running.

### Terminal B — SLAM (container)

```bash
sudo docker start limo_laptop && sudo docker exec -it limo_laptop bash
```

```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
    --params-file /root/maps/slam_params.yaml \
    -p base_frame:=base_link \
    -p use_sim_time:=false
```

The `-p base_frame:=base_link` override forces the correct frame regardless of
what the config file says.

### Terminal C — Nav2 navigation only (container)

```bash
sudo docker exec -it limo_laptop bash
```

```bash
ros2 launch nav2_bringup navigation_launch.py \
    params_file:=/root/maps/nav2.yaml \
    use_sim_time:=false
```

Wait for `Managed nodes are active`. Use `navigation_launch.py`, **not**
`bringup_launch.py`: the latter also starts map_server and AMCL, which would
fight slam_toolbox over `/map` and `map → odom`.

### Terminal D — RViz (container)

```bash
sudo docker exec -it limo_laptop bash
rviz2
```

- Fixed Frame: `map`
- Add: Map (`/map`, Durability: Transient Local), LaserScan (`/scan`,
  Reliability: Best Effort), Map (`/global_costmap/costmap`)
- No 2D Pose Estimate needed — SLAM starts at the map origin automatically
  (the robot's position when Terminal B started).

---

## Running the SLAM demo

1. The map appears small in RViz (just what the LiDAR sees from the start).
2. Click **2D Goal Pose** → the robot navigates **and** the map grows as it explores.
3. Send goals only into white (known free) space — Nav2 won't plan into gray
   unknown space.
4. Revisiting an area triggers **loop closure** — the map snaps to correct drift.

Drive slowly and revisit places you've already mapped: loop closures are what
keep a long map consistent. You can also drive by hand (`ros2 run
teleop_twist_keyboard teleop_twist_keyboard`) instead of sending goals; stop
Terminal C first so only one node publishes `/cmd_vel`.

---

## Save the map

Standard occupancy grid (for AMCL navigation later, see
[`NAVIGATION.md`](NAVIGATION.md)); writes `slam_map.pgm` + `slam_map.yaml`:

```bash
ros2 run nav2_map_server map_saver_cli -f /root/maps/slam_map
```

Serialized pose graph (lets slam_toolbox **continue** this map later); writes
`slam_map.posegraph` + `slam_map.data`:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/root/maps/slam_map'}"
```

Save both. Always save into `/root/maps`: that's the only folder the laptop (and
MATLAB) can see.

## Continue SLAM from a saved map (next session)

Add to `/root/maps/slam_params.yaml` (under `slam_toolbox: ros__parameters:`):

```yaml
    map_file_name: /root/maps/slam_map     # no extension
    map_start_at_dock: true                # start at the saved map's first pose
    mode: mapping
```

slam_toolbox loads the previous pose graph and extends it. Start the robot at the
same spot and heading it had when that map was begun.

Only slam_toolbox's serialized `.posegraph` format can be continued. A plain
`.pgm`/`.yaml` (like `mapMTR5`) cannot be fed back into slam_toolbox.

---

## Troubleshooting

| Symptom | Likely cause → fix |
|---|---|
| No map ever appears; log says it is waiting for a transform | `base_frame` is still `base_footprint` → use the `-p base_frame:=base_link` override |
| Map appears but RViz shows nothing | Map display Durability must be *Transient Local* |
| Map smears / walls double | Driving or turning too fast; slow down and revisit areas for loop closure |
| Nav2 refuses a goal | Goal is in gray (unknown) space or inside inflated walls; pick a white spot |
| `map_saver_cli` times out | slam_toolbox not running / `/map` not visible in this shell (`ros2 topic list`) |
| Saved files not visible on the laptop | Saved outside `/root/maps`; re-save with the full `/root/maps/...` path |

#### Key paths

| What        | Path (in container)             |
| ----------- | ------------------------------- |
| SLAM config | `/root/maps/slam_params.yaml`   |
| Nav2 params | `/root/maps/nav2.yaml`          |
| Saved maps  | `/root/maps/slam_map.*`         |

---

_Part of the [LIMO documentation index](../README.md#documentation) · [repo home](../README.md)._
