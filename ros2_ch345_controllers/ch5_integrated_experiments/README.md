# ch5_integrated_experiments

`ch5_integrated_experiments` 用于第 5 章综合实验：把第 3 章执行层的力/位置协同控制与第 4 章参考层的人机共享导纳/仲裁统一到同一套 Gazebo 在线实验、日志、指标统计、绘图和实物预检流程中。

代码逐模块说明见：

```text
/home/liu/franka_ros2_ws/src/ch5_integrated_experiments/CODE_MODULE_GUIDE.md
```

## 1. 核心命令

构建：

```bash
cd /home/liu/franka_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ch4_shared_gt_controller ch5_integrated_experiments --symlink-install
source install/setup.bash
```

运行默认 Gazebo 综合实验：

```bash
ros2 run ch5_integrated_experiments run_ch5_gazebo_suite \
  --task-mode no_rcm \
  --scenario mixed_sequence \
  --methods standard_impedance,traditional_hybrid,standard_mpc,balanced_fixed,reference_only,execution_only,full_method \
  --trials 1 \
  --timeout-s 95
```

只验证综合方法：

```bash
ros2 run ch5_integrated_experiments run_ch5_gazebo_suite \
  --task-mode no_rcm \
  --scenario mixed_sequence \
  --methods full_method \
  --trials 1 \
  --timeout-s 95
```

补跑安全投影消融：

```bash
ros2 run ch5_integrated_experiments run_ch5_gazebo_suite \
  --task-mode no_rcm \
  --scenario mixed_sequence \
  --methods no_projection \
  --trials 1 \
  --timeout-s 95
```

运行第五章 `T_vio` 专项验证。该实验使用更强、更长的危险法向压入工况，
用于避免普通综合实验中各方法 `T_vio` 全为 0 而无法体现安全投影优势。
在线实验默认使用 `F_min=0.38 N, F_max=1.05 N, force_margin=0.06 N`，
并将 `no_projection` 映射为不使用安全投影、也不使用力裕度门控的强消融基线：

```bash
ros2 run ch5_integrated_experiments run_ch5_tvio_challenge \
  --run-gazebo \
  --methods direct_accept,balanced_fixed,no_projection,reference_only,full_method \
  --trials 3 \
  --task-duration-s 22 \
  --timeout-s 95
```

不启动 Gazebo 时，可先用 deterministic dry-run 检查指标、绘图和报告流程：

```bash
ros2 run ch5_integrated_experiments run_ch5_tvio_challenge \
  --dry-run \
  --methods direct_accept,balanced_fixed,no_projection,reference_only,full_method \
  --trials 1
```

聚合多组实验并导出论文表格：

```bash
ros2 run ch5_integrated_experiments ch5_aggregate_results \
  --inputs /home/liu/franka_ros2_ws/results/ch5_integrated \
  --output-dir /home/liu/franka_ros2_ws/results/ch5_integrated/aggregate \
  --expected-methods balanced_fixed,reference_only,execution_only,no_projection,full_method
```

实物实验前预检 Falcon 和机器人控制接口：

```bash
ros2 run ch5_integrated_experiments ch5_hardware_preflight \
  --publisher-topics /joint_states,/falcon/ee_pose,/falcon/velocity,/falcon/joystick \
  --subscriber-topics /no_rcm_effort_controller/commands,/falcon/force_cmd \
  --timeout-s 10 \
  --output-report /home/liu/franka_ros2_ws/results/ch5_integrated/hardware_preflight.json
```

Falcon 主端位移接入第 5 章控制链：

```bash
ros2 launch ros2_falcon falcon.launch.py
ros2 launch ch5_integrated_experiments ch5_falcon_bridge.launch.py
ros2 launch ch4_shared_gt_controller ch4_shared_gt_gazebo.launch.py \
  scenario:=external \
  arbitration_strategy:=full_method \
  execution_strategy:=dynamic_gt \
  external_human_topic:=/ch5/human_delta
```

## 2. 方法开关

方法开关定义在 `ch5_integrated_experiments/method_switches.py`。

| method | 第 4 章参考层 | 第 3 章执行层 | 用途 |
|---|---|---|---|
| `standard_impedance` | 动态 `alpha_HR` + 安全投影 | 标准阻抗控制 | 标准位置/阻抗执行层基线 |
| `traditional_hybrid` | 动态 `alpha_HR` + 安全投影 | 传统混合力位控制 | 切向位置、法向力的经典解耦基线 |
| `standard_mpc` | 动态 `alpha_HR` + 安全投影 | 标准 MPC/QP 风格约束控制 | 预测约束类执行层基线 |
| `balanced_fixed` | 固定 `alpha_HR=0.5` | 固定 `alpha_FP=0.5` | 固定融合基线 |
| `direct_accept` | 直接接受人侧输入 | 固定 `alpha_FP=0.5` | 危险法向输入上界基线 |
| `reference_only` | 动态 `alpha_HR` + 安全投影 | 固定 `alpha_FP=0.5` | 只验证第 4 章参考层贡献 |
| `execution_only` | 自主参考 | 动态 `alpha_FP` | 只验证第 3 章执行层贡献 |
| `no_projection` | 动态 `alpha_HR` 但关闭安全投影 | 动态 `alpha_FP` | 验证第 4 章安全投影必要性 |
| `full_method` | 动态 `alpha_HR` + 安全投影 | 动态 `alpha_FP` | 第 5 章综合方法 |

## 3. 输出

每次实验会生成：

- `raw/*/result.npz`：统一时序日志。
- `raw/*/summary.json`：单次控制器摘要。
- `ch5_gazebo_metrics.csv`：第 5 章统一指标表。
- `CH5_GAZEBO_INTEGRATED_REPORT.md`：自动报告。
- `figures/ch5_metrics_bars.png`：综合指标柱状图。
- `figures/ch5_representative_timeseries.png`：典型时序图。
- `CH5_TVIO_CHALLENGE_REPORT.md`：`T_vio` 专项验证报告。
- `figures/ch5_tvio_bars.png`、`figures/ch5_tvio_timeseries.png`：`T_vio` 专项验证图。
- `aggregate/ch5_aggregate_metrics.csv`：跨实验统计。
- `aggregate/ch5_paper_table.tex`：论文表格片段。
- `aggregate/CH5_AGGREGATE_REPORT.md`：对比方法可行性和缺口报告。

主要判据：

- 理想 Gazebo 仿真中，末段轨迹误差 RMS 不大于 `3 mm`，最大值不大于 `4.5 mm`。
- 模拟实物实验中，考虑传感器噪声、慢漂、柔性接触刚度不确定性和人输入抖动，末段轨迹误差 RMS 判据放宽为 `10 mm`，最大值判据放宽为 `15 mm`。
- 力越界时间不大于 `0.20 s`。
- `with_rcm` 理想仿真末段 RCM RMS 不大于 `3 mm`；模拟实物实验中末段 RCM RMS 不大于 `10 mm`。

更完整的第 5 章写作与实验方案见 `CH5_EXPERIMENT_MANUAL.md`。
