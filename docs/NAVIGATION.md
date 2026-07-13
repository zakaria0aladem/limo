# Regular nav2 steps 
This is to run the navigation as it is in the manual brovided by the manufacturer with added steps for the setup and an eisear workflow, for a full setup refer to [DEVICE_SETUP.md](DEVICE_SETUP.md)

> This document assums that the map is already made and is saved on the robot, the one brovided in this repo is of the optitrack sectioned area in the MTR lab(ESB 0012) if you want to create your own please follow the steps from the manual and save it at the /maps folder before going any further. 

first make sure both machines are on the same network and the same subnet

```bash
hostname -I
```
### AUS_Wirless

IP: inet 10.25.150.233

to connect ssh

```bash
 ssh agilex@10.25.150.233
```

and to connect display we use:

```bash
 ssh -X agilex@10.25.150.233
```

  
### `GL-BE9300-2a5`
```bash
ssh -X agilex@192.168.8.185
```


they also have to be on the same ros distribution

```bash
xhost +local:docker

sudo docker run -dit \
  --net=host --ipc=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/ros2_ws:/root/ros2_ws \
  -v ~/maps:/root/maps \
  -e ROS_DOMAIN_ID=10 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --name limo_laptop \
  osrf/ros:foxy-desktop
```

```
docker start limo_laptop
```

```
sudo docker exec -it limo_laptop bash
```

Disable firewall (if needed):

```
sudo ufw disable
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

1）First launch the LiDAR. Enter the command in the terminal:

```
ros2 launch limo_bringup limo_start.launch.py
```

2) start the navigation

```
ros2 launch limo_bringup limo_nav2.launch.py
```

or

2) run with no rviz

```bash
ros2 launch limo_bringup limo_nav2.launch.py use_rviz:=false
```

set goal

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}"
```

# Running LIMO Nav2 from Laptop (Daily Workflow)

### Prerequisites (one-time)

setup your device [DEVICE_SETUP.md](DEVICE_SETUP.md)


---

## Step 1 — Start robot drivers (Terminal A)

bash

```bash
ssh agilex@192.168.8.184 #or any IP that is currently in use by both machines
```

```bash
pkill -9 ros2
```

### Important: Run this at every new terminal (ssh and container)
```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
```

launch the robot drivers
```bash
ros2 launch limo_bringup limo_start.launch.py
```

Wait until you see:

- `[limo_base] connect the serial port: '/dev/ttyTHS0'`
- `[YDLIDAR] Now lidar is scanning...`

**Leave this terminal running.** Closing it stops the robot.

---

## Step 2 — Start the laptop container (Terminal B — laptop host)

bash

```bash
xhost +local:docker
sudo docker start limo_laptop
sudo docker exec -it limo_laptop bash
```

Inside the container, verify connection to robot:

bash

```bash
ros2 daemon stop && ros2 daemon start
ros2 topic list
```

Expected: `/cmd_vel`, `/imu`, `/limo_status`, `/odom`, `/scan`, `/tf`, `/tf_static`.

## Step 3 — Launch Nav2 (Terminal B, inside container)

```bash
ros2 launch nav2_bringup bringup_launch.py \
    map:=/root/maps/mapMTR5.yaml \
    params_file:=/root/maps/nav2.yaml \
    use_sim_time:=false
```

Wait for: `Managed nodes are active` (about 10 seconds).

**Leave this terminal running.**

---

## Step 5 — Bootstrap the map frame (Terminal C — new container shell)

Open a new terminal on the laptop host:

bash

```bash
sudo docker exec -it limo_laptop bash
```

Inside, publish a temporary `map → odom` transform so RViz can find the map frame:

bash

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

**Leave this running for now. You'll kill it after AMCL takes over.**

---

## Step 6 — Open RViz (Terminal D — new container shell)

bash

```bash
sudo docker exec -it limo_laptop bash
```

Inside:

bash

```bash
rviz2
```

In RViz:

1. **Global Options → Fixed Frame:** `map`
2. **Add → By topic:**
    - `/map` → Map (set **Durability Policy: Transient Local**)
    - `/scan` → LaserScan (set **Reliability: Best Effort**)
    - `/global_costmap/costmap` → Map (set **Durability Policy: Transient Local**, **Color Scheme: costmap**)
3. The map should be visible

---

## Step 7 — Localize the robot

1. Look at the map and identify where the robot physically is in the room
2. Click **2D Pose Estimate** at the top of RViz
3. **Click + drag** on the map at the robot's real position, pointing the arrow in the direction the robot is facing
4. Release

AMCL now publishes the real `map → odom` transform. The laser scan dots should align with the black walls in the map.

**Adjust if needed:** click 2D Pose Estimate again and refine. Getting the orientation right matters more than position.

---

## Step 8 — Kill the static transform (Terminal C)

Now that AMCL is publishing `map → odom`, kill the fake one:

```
Ctrl+C
```

Map should stay put. If it jumps, click 2D Pose Estimate again.

---

## Step 9 — Send navigation goals

**Via RViz:**

1. Click **2D Goal Pose** at the top
2. Click + drag on the map at the destination, arrow = final facing direction
3. Robot drives

**or Via command line (Terminal C, container):**

bash

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}"
```

---

### Shutdown sequence

1. **Terminal D (RViz):** close window or Ctrl+C
2. **Terminal B (Nav2 launch):** Ctrl+C — wait until all nodes shut down
3. **Terminal A (robot):** Ctrl+C
4. **Container** can be left running or stopped: `sudo docker stop infallible_dhawan` from laptop host

---

### Troubleshooting

**Topics not appearing in container**

bash

```bash
ros2 daemon stop && ros2 daemon start
echo $RMW_IMPLEMENTATION    # must be rmw_fastrtps_cpp
echo $ROS_DOMAIN_ID         # must be 10
unset FASTRTPS_DEFAULT_PROFILES_FILE
```

**Map doesn't show in RViz**

- Map display **Durability Policy** must be `Transient Local`
- Fixed Frame must be `map` (use static_transform_publisher bootstrap if needed)
- Force republish: `ros2 lifecycle set /map_server deactivate && ros2 lifecycle set /map_server activate`

**Robot won't move on goal**

- Check `/cmd_vel` is being published from Nav2: `ros2 topic echo /cmd_vel`
- Check AMCL has a valid pose: laser dots should overlap walls
- Try a closer/simpler goal first

**Deserialization errors return**

- `limo_msgs` workspace not sourced. Inside container: `source /root/ros2_ws/install/setup.bash`

# SLAM

SLAM (building a map with the onboard LiDAR) doc: see **[SLAM.md](SLAM.md)**.

## Nav2 Speed Tuning

Velocity limits for navigation live in `nav2.yaml` under
`controller_server` (DWB controller). Nav2 must be restarted after editing.

There are two speed limits:
1. Robot hardware max (~1.0 m/s for the LIMO)
2. Nav2-allowed max (set in nav2.yaml — this is what actually limits you)

### Parameters that matter

| Param | Default | Purpose |
|---|---|---|
| max_vel_x | 0.22 | top linear speed (m/s) |
| max_speed_xy | 0.44 | speed magnitude cap (must be ≥ max_vel_x) |
| max_vel_theta | 0.8 | top angular speed (rad/s) |
| decel_lim_x | -0.5 | braking — must scale with speed |
| acc_lim_theta | 0.2 | angular acceleration |
| controller_frequency | 10.0 | control loop rate |
| inflation_radius | 0.02 | wall clearance buffer (costmap) |
| sim_time | 1.5 | DWB trajectory look-ahead time |
| xy_goal_tolerance | 0.05 | goal arrival tolerance |

#### Apply (sed — adjust the "from" value to current file contents)
```bash
sed -i 's/max_vel_x: 0.22/max_vel_x: 0.80/' /root/maps/nav2.yaml
sed -i 's/max_speed_xy: 0.44/max_speed_xy: 0.80/' /root/maps/nav2.yaml
sed -i 's/decel_lim_x: -0.5/decel_lim_x: -2.0/' /root/maps/nav2.yaml
sed -i 's/controller_frequency: 10.0/controller_frequency: 20.0/' /root/maps/nav2.yaml
sed -i 's/inflation_radius: 0.02/inflation_radius: 0.25/' /root/maps/nav2.yaml
sed -i 's/sim_time: 1.5/sim_time: 2.0/' /root/maps/nav2.yaml
sed -i 's/xy_goal_tolerance: 0.05/xy_goal_tolerance: 0.15/' /root/maps/nav2.yaml
```

Verify:
```bash
grep -E "max_vel_x:|max_speed_xy:|decel_lim_x:|controller_frequency:|inflation_radius:|sim_time:|xy_goal_tolerance:" /root/maps/nav2.yaml
```

---

## Rules and warnings

> Scale supporting params with speed
> Raising max_vel_x alone makes the robot lurch and overshoot. Deceleration,
> controller frequency, inflation radius, and sim_time must all scale up too.

> LIMO hardware limit ~1.0 m/s
> Setting max_vel_x above ~1.0 does nothing — motors saturate. Safe indoor
> ceiling is ~0.6–0.7 m/s; 0.8 is aggressive.

> Turning radius
> min turning radius = max_vel_x / max_vel_theta
> At 0.8 / 0.8 = 1.0 m. If the robot can't corner, raise max_vel_theta.

> sed only replaces exact matches
> If a sed command "does nothing", the value was already changed in a prior
> run. Always `grep` current values first, then target those exact numbers.

## Measure actual speed
Send a goal down a long straight path, then watch reported velocity:
```bash
ros2 topic echo /odom --field twist.twist.linear.x
```
Peak value = real top speed.

https://www.mathworks.com/help/releases/R2023a/pdf_doc/supportpkg/turtlebotrobot/turtlebotrobot_ug.pdf

---

_Part of the [LIMO documentation index](../README.md#documentation) · [repo home](../README.md)._
