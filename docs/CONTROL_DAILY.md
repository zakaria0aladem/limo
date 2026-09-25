# Control research — daily manual

**Goal each session:** close the control loop: mocap → controller (P/PID/LQR) →
`/cmd_vel`. **Nav2 OFF.** The one-time setup is in
[`CONTROL_SETUP.md`](CONTROL_SETUP.md); this is the fast per-session run.

**Only ONE publisher on `/cmd_vel`.** This model drives `/cmd_vel` directly.
**Stop the Nav2 stack (and the wandering node) first**, or they and Simulink
fight over the robot. Keep the robot drivers and `vrpn_mocap` running.

**Terminal map (control workflow: 2 terminals + MATLAB).** **A** robot (SSH) ·
**B** vrpn_mocap (container). No Nav2, no map_server, no RViz needed. MATLAB runs
the model.

---

## Terminal A — Robot drivers (SSH)

```bash
ssh agilex@192.168.8.185        # verify the IP with hostname -I
pkill -9 ros2
```

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
ros2 launch limo_bringup limo_start.launch.py
```

Wait for `Now lidar is scanning...`. Leave running.

## Terminal B — Mocap bridge (container)

```bash
sudo docker start limo_laptop
sudo docker exec -it limo_laptop bash
```

```bash
echo $FASTRTPS_DEFAULT_PROFILES_FILE      # must print /root/maps/fastdds_udp.xml (DDS fix)
# if empty (not in ~/.bashrc):  export FASTRTPS_DEFAULT_PROFILES_FILE=/root/maps/fastdds_udp.xml
ros2 launch vrpn_mocap client.launch.yaml server:=192.168.8.184 port:=3883
```

Wait for `Created new tracker Limo`. Leave running.

**Do NOT start Nav2.** The comparison/navigation workflow uses the Nav2 bundle;
the **control** workflow does not. The Simulink model IS the controller.

---

## MATLAB

```matlab
% if not in startup.m, this MUST run before any ros2 object (else restart MATLAB):
setenv("FASTRTPS_DEFAULT_PROFILES_FILE", fullfile(getenv('HOME'), "maps", "fastdds_udp.xml"));

limo_ctrl_params            % load params + gains (prints the LQR K)
build_limo_control_model    % only if the .slx isn't built yet, or the build script changed
open_system('limo_mocap_control')
```

If the model is already built and saved, just `limo_ctrl_params` then
`open_system('limo_mocap_control')`; no need to rebuild. The model reads `P`,
`CTRL` and the variant objects from the base workspace, so **always run
`limo_ctrl_params` first** in a fresh MATLAB session.

**In the model (check once per fresh MATLAB session):**

- Simulation → ROS Toolbox → **ROS Network** → domain **10**, `rmw_fastrtps_cpp`.

---

## Run sequence (every time)

1. **Set the goal + speed** in `limo_ctrl_params.m`, then re-run it:

   ```matlab
   P.goal  = [0.5; 0; 0];   % [x; y; theta_rad] in the mocap/map frame: start close and clear
   P.v_max = 0.15;          % start slow
   ```

   Re-run `limo_ctrl_params` after any edit so the base workspace updates.

2. **Pick the controller** (in `limo_ctrl_params.m`, then re-run it):

   ```matlab
   CTRL = 1;    % 1 = P,  2 = PID,  3 = LQR
   ```

   The active variant is chosen when the simulation starts, so changing `CTRL`
   needs no rebuild; just Stop and Run again. (Changing LQR's `Q`/`R`/`v0` *does*
   need a rebuild, because K is baked into the block.)

3. **Press Run.** The robot does NOT move yet (E-STOPs are safe).
4. **Confirm sensing:** push the robot by hand → the `x`, `y`, `theta` displays
   track it.
5. **Enable motion:** double-click **both** E-STOP switches. Keep a finger ready
   to double-click them back.
6. **Watch:** the XY Graph traces the path; the `v,w` scope shows the commands.
   The robot converges on the goal and stops.
7. **Stop:** double-click the E-STOPs back to zero **first**, then Stop the sim.

---

## Swapping controllers (the research loop)

| `CTRL` | Controller | Note |
|---|---|---|
| 1 | P go-to-goal | simplest; a good first test |
| 2 | PID | anti-windup included |
| 3 | LQR | gain `K` from the Riccati equation, baked in at build |

Change `CTRL`, re-run `limo_ctrl_params`, Run. Same interface, same plant, same
rate → a fair comparison. Record each run to compare them offline (see
[CONTROL_SETUP §8](CONTROL_SETUP.md#8-analysis-tools-control-system-toolbox-comparing-controllers)).

**Trajectory tracking (a harder benchmark).** To track a moving reference instead
of a fixed point, replace the `Goal` Constant block with a signal generating
`[xd(t); yd(t); θd(t)]` (circle, figure-eight). No planner needed. This is the
more standard control-research task.

---

## Read the robot state anytime (separate MATLAB, optional)

```matlab
h = limo_connect(domainID=10);
s = limo_state(h);      % prints truth / estimate / odom / velocity / battery + drift
```

Useful for sanity-checking the pose while tuning. With Nav2 off, the "estimate"
row (`map → base_link`) is empty: nothing publishes `map → odom` in this workflow.

---

## Shutdown

1. E-STOPs → zero, then Stop the sim.
2. `Ctrl+C` Terminal B (vrpn_mocap).
3. `Ctrl+C` Terminal A (robot).

If the robot is still rolling after the sim stops:
`ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"` from a container
shell.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| Subscribe block / MATLAB gets no pose | QoS: **best-effort** on the block; domain 10; the DDS profile set in the container shell **and** in MATLAB before any ros2 object |
| MATLAB sees robot topics but not the container's | shared-memory / DDS: the `fastdds_udp.xml` fix on both sides ([CONTROL_SETUP §3](CONTROL_SETUP.md#3-the-dds-fix-one-time-file-used-every-session)) |
| Displays show nothing | Mocap not flowing: is `vrpn_mocap` up? Is `Limo` tracked in Motive? |
| `Undefined variable P` / `CTRL` when pressing Run | Run `limo_ctrl_params` first |
| Robot won't move | E-STOP switches still at zero: double-click them |
| Robot lurches / oscillates | Lower `P.Kp_rho`, `P.Kp_alpha`, `P.v_max`; check `P.Ts` matches the Subscribe sample time (0.05) |
| Two things driving the robot | Nav2 or the wandering node still running: stop it |
| Model runs faster than real time | Enable Run → **Simulation Pacing**, or add a Simulation Rate Control block |
| `build_...` stops on a block parameter | Set that one field by hand in the block dialog (release naming drift), then re-run |
| Model behaves like an old version | The `.slx` was built by an older script: run `build_limo_control_model` again |

---

_Part of the [LIMO documentation index](../README.md#documentation) · [repo home](../README.md)._
