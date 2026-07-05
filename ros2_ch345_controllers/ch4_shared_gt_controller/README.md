# ch4_shared_gt_controller

Chapter 4 ROS 2 Gazebo arbitration package cloned from `shared_gt_controller`.
The original package is left unchanged; this clone adds the Chapter 4 reference
layer experiment fields and strategy replay controls.

Current scope:

- `task_mode:=no_rcm`: primary Chapter 4 flexible-contact arbitration replay.
- `task_mode:=with_rcm`: retained from the cloned controller for boundary tests.
- `controller_variant:=gt_kf`: 0213-style game-theoretic gain with fuzzy/KF
  human-robot arbitration.
- `arbitration_strategy:=full_method`: dynamic alpha_HR, safety projection,
  tangential/normal split, beta, kappa_N and rho_F logging.
- `arbitration_strategy:=direct_accept`: baseline that accepts teleoperation
  reference directly.
- `scenario:=mixed_sequence`: tangential correction, short pulse, unsafe normal
  push and sustained intervention in one repeatable sequence.

Run:

```bash
source /opt/ros/humble/setup.bash
source /home/liu/franka_ros2_ws/install/setup.bash
ros2 launch ch4_shared_gt_controller ch4_shared_gt_gazebo.launch.py \
  task_mode:=with_rcm controller_variant:=gt_kf
```

No-RCM run:

```bash
source /opt/ros/humble/setup.bash
source /home/liu/franka_ros2_ws/install/setup.bash
ros2 launch ch4_shared_gt_controller ch4_shared_gt_gazebo.launch.py \
  task_mode:=no_rcm controller_variant:=gt_kf
```

Chapter 4 default run:

```bash
source /opt/ros/humble/setup.bash
source /home/liu/franka_ros2_ws/install/setup.bash
ros2 launch ch4_shared_gt_controller ch4_shared_gt_gazebo.launch.py \
  task_mode:=no_rcm controller_variant:=gt_kf \
  arbitration_strategy:=full_method scenario:=mixed_sequence
```

Plot:

```bash
ros2 run ch4_shared_gt_controller plot_shared_gt_result --input <run_dir>
```

Recompute Chapter 4 window metrics and compare strategies:

```bash
ros2 run ch4_shared_gt_controller recompute_ch4_metrics --input <run_dir>
ros2 run ch4_shared_gt_controller compare_ch4_results \
  --runs <run_dir_1> <run_dir_2> --recompute-window-metrics
```

Run a repeatable Gazebo experiment matrix:

```bash
ros2 run ch4_shared_gt_controller run_ch4_gazebo_suite \
  --scenarios mixed_sequence \
  --strategies autonomous_only direct_accept fixed_blend single_sigmoid dynamic_no_projection full_method
```

Export paper figures, tables and data copies:

```bash
ros2 run ch4_shared_gt_controller export_ch4_paper_assets \
  --runs <run_dir_1> <run_dir_2> --comparison-dir <comparison_dir>
```

The controller uses the same `no_rcm_effort_controller` effort command topic as
the existing ROS 2 no-RCM package.

Code documentation:

- `CH4_CONTROLLER_CODE_GUIDE.md`: module overview, method explanations and
  thesis-variable mapping.
- `CH4_EXPERIMENT_MANUAL.md`: experiment design, metrics and run commands.

Verified Chapter 4 Gazebo runs:

- full method: `results/ch4_shared_gt_gazebo/no_rcm_gt_kf_full_method_mixed_sequence_20260702_035207`,
  tracking last-window RMS 2.81 mm, no force violation.
- direct accept baseline: `results/ch4_shared_gt_gazebo/no_rcm_gt_kf_direct_accept_mixed_sequence_20260702_034702`,
  tracking last-window RMS 2.58 mm, force violation time 0.72 s.
