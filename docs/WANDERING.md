# Reactive wandering (potential fields) — `limo_nav`

The simplest autonomy in this repo: no map, no localization, no Nav2. A single
node reacts to the laser scan in real time. Useful as a first bring-up test —
if this drives smoothly and avoids walls, the robot, `/scan`, and `/cmd_vel`
are all healthy.

## How it works

The node (`limo_nav/wandering.py`, class `PotentialField`) sums two vectors in
the robot body frame:

- **Attraction** — a constant vector pulling the robot forward (`V_attraction`).
- **Repulsion** — built from `/scan`: every return closer than 0.6 m pushes the
  robot away, weighted by `1/range`, summed over all beams.

The resultant vector's magnitude sets linear speed and its angle sets angular
speed:

```
F = attraction + repulsion
v_lin = |F|                     (0 if F points backwards, i.e. F_x < 0)
v_ang = atan2(F_y, F_x)         (rad, in [-pi, pi])

cmd_vel.linear.x  = v_lin / 250
cmd_vel.angular.z = v_ang / 4 * PI     # Python precedence: (v_ang / 4) * PI
```

It also publishes the attraction, repulsion, and final vectors as `PoseStamped`
on `/attraction_vector`, `/repulsion_vector`, `/final_vector` so you can
visualize them in RViz.

## Build (once)

The package is in this repo at `src/limo_nav`. On the laptop host, copy it into
the mounted workspace; then build it in the container:

```bash
cp -r ~/limo/src/limo_nav ~/ros2_ws/src/            # host
```

```bash
cd /root/ros2_ws && colcon build --packages-select limo_nav   # container
source install/setup.bash
```

## Run

```bash
# robot drivers up first (DEVICE_SETUP step 1, LiDAR publishing /scan), then in the container:
ros2 run limo_nav wandering
```

Stop it with `Ctrl+C`. The node doesn't send a zero command on exit, so if the
robot keeps rolling:
`ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"`.

## Tuning

The gains are inline in `controller()` and `scan_callback()`:

- **The active values are the simulation ones.** `V_attraction = [30.0, 0.0]`,
  `v_lin / 250`, `v_ang / 4 * PI` are live; the commented "Real robot" lines
  (`[10.0, 0.0]`, `/3400`, `/6 * PI`) are the hardware alternatives. With the
  sim values and no obstacle, the robot drives at 30/250 = 0.12 m/s. Swap to the
  real-robot lines (or scale down) before trying it near people or walls.
- `V_attraction`: bigger = more forward drive.
- The `v_lin` and `v_ang` scale factors set the final speeds. Note that
  `v_ang / 4 * PI` is `(v_ang/4)·π`, up to about 2.5 rad/s for an obstacle right
  beside the robot. Start slow on hardware.
- The `0.6 m` / `0.08 m` window in `scan_callback` sets how close an obstacle
  must be to repel, and rejects spurious near-zero returns.

> Only one node may own `/cmd_vel`. Don't run this alongside Nav2 or the
> Simulink controller.

---

_Part of the [LIMO documentation index](../README.md#documentation) · [repo home](../README.md)._
