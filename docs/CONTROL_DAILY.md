# Control research daily
# Control Research Platform — Daily Manual

> [!abstract] Goal each session Close the control loop: mocap → controller (P/PID/LQR) → `/cmd_vel`. **Nav2 OFF.** First-time setup is in Control Research — First Setup — this note is the fast per-session run.

> [!warning] Only ONE publisher on /cmd_vel This model drives `/cmd_vel` directly. **Stop the Nav2 stack first**, or Nav2 and Simulink fight over the robot. Keep robot drivers + `vrpn_mocap` running.

---

## Terminal A — Robot drivers (SSH)

bash

```bash
ssh agilex@192.168.8.185
ros2 launch limo_bringup limo_start.launch.py
```

Wait for `lidar is scanning`. Leave running.

## Terminal B — Mocap bridge (container)

bash

```bash
sudo docker start limo_laptop
sudo docker exec -it limo_laptop bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/root/maps/fastdds_udp.xml   # DDS fix — every new shell!
ros2 launch vrpn_mocap client.launch.yaml server:=192.168.8.184 port:=3883
```

Wait for `Created new tracker Limo`. Leave running.

> [!note] Do NOT start Nav2 The comparison/nav workflow uses the Nav2 bundle. The **control** workflow does not — the Simulink model IS the controller.

---

## MATLAB

matlab

```matlab
% if not in startup.m, this MUST run before any ros2 object:
setenv("FASTRTPS_DEFAULT_PROFILES_FILE","/home/zakaria/maps/fastdds_udp.xml");

limo_ctrl_params            % load params + gains (prints LQR K)
build_limo_control_model    % (only if the .slx isn't already built/saved)
open_system('limo_mocap_control')
```

If the model is already built and saved, just `open_system('limo_mocap_control')` — no need to rebuild.

**In the model (once per fresh MATLAB session):**

- Simulation → ROS Toolbox → **ROS Network** → domain **10**, `rmw_fastrtps_cpp`.

---

## Run sequence (every time)

1. **Set the goal + speed** in `limo_ctrl_params.m`, then re-run it:

matlab

```matlab
   P.goal  = [0.5; 0; 0];   % start close and clear
   P.v_max = 0.15;          % start slow
```

(Re-run `limo_ctrl_params` after any edit so the base workspace updates.)

2. **Pick the controller:**

matlab

```matlab
   CTRL = 1    % 1 = P,  2 = PID,  3 = LQR
```

Re-run `limo_ctrl_params`, then rebuild OR just change it live if the model reads `CTRL`.

3. **Press Run.** Robot does NOT move yet (E-STOPs safe).
4. **Confirm sensing:** push the robot by hand → `x`,`y`,`theta` displays track it.
5. **Enable motion:** double-click **both** E-STOP switches. Keep a finger ready to double-click them back.
6. **Watch:** XY Graph traces the path; `v,w` scope shows commands. Robot converges on the goal and stops.
7. **Stop:** double-click E-STOPs back to zero, then Stop the sim.

---

## Swapping controllers (the research loop)

|`CTRL`|Controller|Note|
|---|---|---|
|1|P go-to-goal|simplest; good first test|
|2|PID|anti-windup included|
|3|LQR|gain `K` from Riccati, baked in at build|

Change `CTRL`, re-run `limo_ctrl_params`, rebuild (or re-run), Run. Same interface, same plant, same rate → fair comparison.

> [!tip] Trajectory tracking (harder benchmark) To track a moving reference instead of a fixed point: replace the `Goal` Constant block with a signal generating `[xd(t); yd(t); θd(t)]` (circle, figure-eight). No planner needed. This is the more standard control-research task.

---

## Read robot state anytime (separate MATLAB, optional)

matlab

```matlab
h = limo_connect(domainID=10);
s = limo_state(h);      % prints truth / estimate / odom / velocity / battery + drift
```

Useful to sanity-check pose and see truth-vs-estimate drift while tuning.

---

## Shutdown

1. E-STOPs → zero, Stop sim.
2. Ctrl+C Terminal B (vrpn_mocap).
3. Ctrl+C Terminal A (robot).

---

## Troubleshooting

|Symptom|Fix|
|---|---|
|Subscribe block / MATLAB gets no pose|QoS: **best-effort** on the block; check domain 10; **DDS `export` set** in the container shell + MATLAB `setenv` before ros2 objects|
|Displays show nothing|mocap not flowing — is `vrpn_mocap` up? Is `Limo` tracking in Motive?|
|Robot won't move|E-STOP switches still zero — double-click them|
|Robot lurches / oscillates|lower `P.Kp_rho`, `P.Kp_alpha`, `P.v_max`; check `Ts` matches Subscribe sample time (0.05)|
|Two things driving robot|Nav2 still running — stop it|
|Model runs faster than real time|enable Run → **Simulation Pacing**, or add the Simulation Rate Control block|
|`build_...` warns on a mask param|set that one field by hand in the block dialog (release naming drift)|
|MATLAB sees robot topics but not container's|shared-memory/DDS — the `fastdds_udp.xml` fix, both sides → MATLAB_DDS_Fix|

> [!note] Terminal map (control workflow — just 2 + MATLAB) **A** robot (SSH) · **B** vrpn_mocap (container). No Nav2, no map_server, no RViz needed. MATLAB runs the model.

## References

Control research first setup
