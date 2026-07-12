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
v = |attraction + repulsion|   (clamped to 0 if the x-component is negative)
w = atan2(resultant_y, resultant_x)
```

It also publishes the attraction, repulsion, and final vectors as `PoseStamped`
on `/attraction_vector`, `/repulsion_vector`, `/final_vector` so you can
visualize them in RViz.

## Run

```bash
# robot drivers up first (LiDAR publishing /scan), then:
ros2 run limo_nav wandering
```

## Tuning

The gains are inline in `controller()` and `scan_callback()`:

- `V_attraction` — bigger = more forward drive. There are commented "real robot"
  values (`[10.0, 0.0]`) distinct from the simulation values (`[30.0, 0.0]`).
- The `v_lin / 250` and `v_ang / 4 * PI` scale factors set the final speeds —
  the commented `/3400` and `/6` lines are the real-robot equivalents. Start
  slow on hardware.
- The `0.6 m` / `0.08 m` window in `scan_callback` sets how close an obstacle
  must be to repel, and rejects spurious near-zero returns.

> Only one node may own `/cmd_vel`. Don't run this alongside Nav2 or the
> Simulink controller.

---

_Part of the [LIMO documentation index](../README.md#documentation) · [repo home](../README.md)._
