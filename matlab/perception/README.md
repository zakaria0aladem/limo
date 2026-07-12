# matlab/perception — LiDAR obstacle detection (MATLAB)

Pure-MATLAB obstacle detection off the LIMO's LiDAR (`/scan`). No ROS node to
deploy — MATLAB subscribes to `/scan` directly. This is the "obstacle layer"
first step: report the nearest obstacle's distance and angle, plus whether the
front / left / right sectors are clear. It's the foundation a later avoidance
layer can build on.

## Files

| File | What it does |
|---|---|
| `limo_lidar_obstacle.m` | Function: reads one scan, returns nearest obstacle + sector clearances |
| `limo_lidar_demo.m` | Live polar-plot viewer of the scan, nearest obstacle, and sector status |

## Usage

Requires a connection handle from `matlab/goals/limo_connect.m`:

```matlab
h = limo_connect(domainID=10);

% one-shot read:
obs = limo_lidar_obstacle(h);
%  LiDAR nearest: 0.42 m at -75 deg (right)   <-- OBSTACLE
%    front CLEAR (1.83 m) | left CLEAR (2.10 m) | right BLOCKED (0.42 m)

% live viewer (Ctrl+C to stop):
limo_lidar_demo
```

## Returned struct

```
obs.distance      nearest valid range [m]        (Inf if none)
obs.angle_deg     angle of the nearest obstacle  (+ left, - right, 0 ahead)
obs.side          'front'|'left'|'right'|'behind'|'none'
obs.is_obstacle   nearest < threshold
obs.front/left/right .clear .min_dist   per-sector clearance
obs.scan.ranges / .angles               raw arrays (for plotting)
obs.ok            message received?
```

## Conventions & notes

- **Angle convention (REP-103):** 0° = straight ahead, + = left, − = right.
  So −75° means 75° to the right.
- **Sectors:** front = |angle| ≤ 30°, left = +30…+90°, right = −30…−90°.
- **QoS:** the subscriber is best-effort (the LiDAR publishes best-effort; a
  reliable subscriber would receive nothing).
- **Verify on hardware:** confirm the scan topic is `/scan` and that 0° really
  points forward (`ros2 topic echo /scan --once`). If the sensor's zero isn't
  the robot's front, subtract that constant offset from `angle_deg`.
- **Threshold:** default 0.5 m; pass `threshold=` to change it.
