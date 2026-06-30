# 0608controller: Chapter 5 Dual-Arbitration Simulation

This folder contains the Chapter 5 simulation code used by the thesis package
in `../0608thesis`.

## Main Command

```bash
cd /home/liu/franka_ws_1101
MPLCONFIGDIR=/tmp/matplotlib-codex \
python3 src/panda_robot/tests/0608controller/run_ch5_dual_arbitration.py \
  --backend gazebo --tune --seed 11
```

The script always runs the deterministic analytic simulation.  When
`--backend gazebo` is selected, it also probes the local ROS/Gazebo master and
writes the diagnostic result to `gazebo_status.json`.

## Latest Validated Output

Latest result used for the thesis text:

```text
src/panda_robot/tests/0608controller/results/ch5_dual_arbitration_20260608_215915
```

Important files:

- `metrics.csv`: fixed-alpha and dual-arbitration metrics.
- `tuning_history.csv`: automatic dual-parameter sweep.
- `metrics_bars.png`: summary bars.
- `force_alpha_stiffness_timeseries.png`: force, alpha, and stiffness response.
- `trajectory_xy.png`: tangential human correction and defect avoidance.
- `unsafe_normal_push_detail.png`: unsafe normal-push response.
- `dual_alpha_khat_response.png`: alpha response to estimated stiffness and force margin.

## Gazebo Status

A headless `panda_gazebo panda_world.launch` probe was attempted from Codex.
The first sandboxed run failed because `roslaunch` could not enumerate local
network interfaces.  With elevated permissions ROS master started, but Gazebo
did not stabilize: `spawn_model` reported `entity already exists`, which
indicates a stale or already running `panda` model in the external Gazebo
environment.  The final script records this status instead of killing external
ROS/Gazebo processes, because those processes may belong to a user session or a
real-robot workflow.

To run against a clean Gazebo instance manually:

```bash
source /home/liu/franka_ws_1101/devel/setup.bash
roslaunch panda_gazebo panda_world.launch gui:=false use_custom_action_servers:=false start_moveit:=false load_gripper:=false
```

Then run the main command above in another terminal.
