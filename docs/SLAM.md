# SLAM — build a map with the onboard LiDAR (slam_toolbox)

Online SLAM with `slam_toolbox`: drive the robot to *build* a map while it
navigates, with live loop closure. Save the result and reuse it for AMCL
navigation (`NAVIGATION.md`).

> Prerequisite: the shared base setup in `DEVICE_SETUP.md` (robot bring-up,
> container, Nav2). The mapping parameters are committed at
> `config/slam_params.yaml` (already carries the `base_frame: base_link` fix);
> the "one-time setup" below shows how that file was derived from the
> slam_toolbox default, and the daily workflow's `-p base_frame:=base_link`
> override enforces it regardless.

Online SLAM (slam_toolbox) runs on top of Nav2 so the robot maps an area
AND navigates at the same time.

## One-time setup (in container)

Install slam_toolbox:
```bash
apt update
apt install -y ros-foxy-slam-toolbox
```

Copy the online async config:
```bash
mkdir -p /root/limo_workspace/slam
cp /opt/ros/foxy/share/slam_toolbox/config/mapper_params_online_async.yaml \
   /root/limo_workspace/slam/slam_params.yaml
```

### Critical config edit
The default config uses `base_footprint`, but the LIMO uses `base_link`.
```bash
sed -i 's/base_footprint/base_link/g' /root/limo_workspace/slam/slam_params.yaml
grep base_frame /root/limo_workspace/slam/slam_params.yaml   # must show base_link
```

> [!note] Config not persistent
> `/root/limo_workspace/` is NOT a mounted volume — it lives only inside the
> container. Copy slam_params.yaml to `/root/maps/` (which IS mounted) to
> survive container recreation.

---

## Daily SLAM workflow (4 terminals)

### Terminal A — Robot drivers (SSH)
```bash
ssh agilex@192.168.8.184
ros2 launch limo_bringup limo_start.launch.py
```

### Terminal B — SLAM (container)
```bash
source /opt/ros/foxy/setup.bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
    --params-file /root/limo_workspace/slam/slam_params.yaml \
    -p base_frame:=base_link
```
The `-p base_frame:=base_link` override forces the correct frame regardless
of what the config file says.

### Terminal C — Nav2 navigation only (container)
```bash
source /opt/ros/foxy/setup.bash
ros2 launch nav2_bringup navigation_launch.py \
    params_file:=/root/maps/nav2.yaml \
    use_sim_time:=false
```
Wait for `Managed nodes are active`.

### Terminal D — RViz (container)
```bash
rviz2
```
- Fixed Frame: `map`
- Add: Map (`/map`, Durability: Transient Local), LaserScan (`/scan`,
  Reliability: Best Effort), Map (`/global_costmap/costmap`)
- No 2D Pose Estimate needed — SLAM starts at map origin automatically

---

## Running the SLAM demo
1. Map appears small in RViz (just what lidar sees from start)
2. Click **2D Goal Pose** → robot navigates AND map grows as it explores
3. Send goals only into white (known free) space — Nav2 won't plan into
   gray unknown
4. Revisiting an area triggers **loop closure** — map snaps to correct drift

---

## Save the map

Standard occupancy grid (for navigation use later):
```bash
ros2 run nav2_map_server map_saver_cli -f /root/maps/slam_map
```

Serialized format (preserves pose graph, lets SLAM continue later):
```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/root/maps/slam_map'}"
```

## Continue SLAM from a saved map (next session)
Add to `slam_params.yaml`:
```yaml
    map_file_name: /root/maps/slam_map
    mode: mapping
```
slam_toolbox loads the previous pose graph and extends it.

> [!note]
> Only slam_toolbox's serialized `.posegraph` format can be continued.
> A plain `.pgm`/`.yaml` (like the old mapMTR5) cannot be fed back in.

---

#### Key paths
| What        | Path (in container)                          |
| ----------- | -------------------------------------------- |
| SLAM config | `/root/limo_workspace/slam/slam_params.yaml` |
| Nav2 params | `/root/maps/nav2.yaml`                       |
| Maps        | `/root/maps/`                                |
|             |                                              |
