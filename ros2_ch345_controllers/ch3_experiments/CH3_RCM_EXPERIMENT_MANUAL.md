# 第三章 RCM 约束实验 ROS2/Gazebo 手册

生成日期：2026-07-02

本文档说明 `/home/liu/franka_ros2_ws/src/ch3_experiments` 中新增的 RCM 约束实验代码。第三章主包只负责编排实验、分析数据和生成论文图表；实际在线控制器复用已经从 ROS1 迁移到 ROS2 的 `ch3_controller run_with_rcm`，Gazebo 机器人启动复用 `rcm_bringup`。

## 1. 论文目标

RCM 约束实验用于验证第三章力位协同控制器在带穿刺点几何约束任务中的有效性。相比 no-RCM 扫描，RCM 任务额外要求工具轴线始终穿过 trocar 点：

```text
e_rcm = || p_trocar - projection(p_trocar, line(p_flange, p_tool)) ||
```

控制器仍保持第三章执行层力位协同结构：

```text
alpha_FP = 1   更偏位置、姿态和 RCM 几何保持
alpha_FP = 0   更偏法向接触力跟踪
```

在 RCM 约束下，tool-tip 参考先通过几何杠杆映射到 flange 参考，再由 Pinocchio Jacobian 映射为关节力矩。

## 2. 新增代码

```text
ch3_experiments/analyze_ch3_rcm_result.py
ch3_experiments/gazebo_rcm_adapter.py
ch3_experiments/run_ch3_rcm_gazebo_suite.py
CH3_RCM_EXPERIMENT_MANUAL.md
```

命令入口：

```text
ros2 run ch3_experiments run_ch3_rcm_gazebo_suite
ros2 run ch3_experiments analyze_ch3_rcm_result
```

## 3. 构建

```bash
source /opt/ros/humble/setup.bash
cd /home/liu/franka_ros2_ws
colcon build --packages-select ch3_controller rcm_bringup ch3_experiments --symlink-install
source install/setup.bash
```

## 4. Gazebo 可视化验证

推荐先运行单策略单试次：

```bash
source /opt/ros/humble/setup.bash
source /home/liu/franka_ros2_ws/install/setup.bash

ros2 run ch3_experiments run_ch3_rcm_gazebo_suite \
  --strategies fixed_05 \
  --trials 1 \
  --controller-mode pareto_iter \
  --output-dir /home/liu/franka_ros2_ws/results/ch3_experiments_rcm_gazebo
```

该命令会执行：

1. 启动官方 `franka_gazebo_bringup gazebo_franka_arm_example_controller.launch.py`。
2. 等待 `no_rcm_effort_controller` active。该 controller 只是 7 关节 effort 后端，RCM 几何由 `run_with_rcm` 内部完成。
3. 运行 `ch3_controller run_with_rcm`。
4. 使用虚拟 Kelvin-Voigt 接触环境生成接触力。
5. 生成 RCM 约束指标、图表和 Markdown 报告。

输出目录示例：

```text
results/ch3_experiments_rcm_gazebo/
  gazebo_rcm_launch.log
  rcm_YYYYMMDD_HHMMSS/
    fixed_0.5_t00.npz
    approach_debug/
    ch3_rcm_analysis/analysis/
      ch3_rcm_trial_metrics.csv
      ch3_rcm_strategy_summary.csv
      ch3_rcm_metric_bars.png
      ch3_rcm_representative_timeseries.png
      ch3_rcm_analysis_report.md
```

## 5. 指标

论文建议报告以下指标：

```text
rcm_rmse_mm
rcm_peak_mm
rcm_soft_violation_time_s
track_rmse_mm
track_peak_mm
force_rmse_N
force_jitter_N
alpha_mean / alpha_min / alpha_max
alpha_khat_corr
k_hat_mean_Npm
```

当前分析器按控制器配置使用两个 RCM 参考阈值：

```text
RCM soft limit: 3.5 mm
RCM pause/recovery limit: 6.5 mm
```

## 6. 对比实验设计

建议按由小到大的顺序做：

```bash
ros2 run ch3_experiments run_ch3_rcm_gazebo_suite --strategies fixed_05 --trials 1
```

```bash
ros2 run ch3_experiments run_ch3_rcm_gazebo_suite --strategies fixed_02,fixed_05,fixed_08 --trials 1
```

```bash
ros2 run ch3_experiments run_ch3_rcm_gazebo_suite \
  --strategies fixed_02,fixed_05,fixed_08,continuous_force_margin \
  --trials 1
```

论文中可将 `fixed_02/fixed_05/fixed_08` 作为固定优先级基线，将 `continuous_force_margin` 作为根据力安全裕度和刚度估计自适应变化的合作博弈策略。

## 7. 稳定性注意事项

- 先确认 `no_rcm_effort_controller` 已 active，再启动 RCM 控制器。
- RCM 任务初始位姿必须先经过 `safe_move_to_joint_position(INIT_JOINTS)`，避免工具轴线从不可行姿态开始。
- 接近阶段使用 `alpha=1`，优先保证 free-space 位置和 RCM 几何约束。
- 正式扫描阶段如果 RCM 误差接近软阈值，应提高 `alpha_FP` 并降低扫描速度。
- RCM 误差超过恢复阈值时应暂停 x 向推进，并轻微卸载法向接触，避免在接触中拖动工具轴。
- 若 Gazebo 中启动后明显下坠，先检查 Pinocchio 重力补偿参数和 effort controller 是否 active；不要直接提高扫描控制增益。

## 8. 论文撰写建议

第三章 RCM 小节建议按以下逻辑写：

1. 给出 RCM 几何误差定义和 tool-to-flange 映射。
2. 说明 RCM 下仍使用第三章力位协同博弈控制器，区别是任务空间反馈从 tool-tip 扩展为 flange-space 几何控制。
3. 展示 Gazebo 可视化实验设置：FR3、trocar 点、虚拟分段刚度环境、期望接触力。
4. 对比固定优先级和动态优先级策略。
5. 报告 RCM 误差、工具跟踪误差、力跟踪误差、alpha 变化趋势和稳定性。
6. 讨论 RCM 约束带来的 trade-off：当 RCM 误差升高时，控制器牺牲一部分力跟踪或扫描速度，优先保持穿刺点安全。

## 9. 2026-07-02 本地 Gazebo 验证记录

最新验证报告：

```text
/home/liu/franka_ros2_ws/src/ch3_experiments/CH3_RCM_GAZEBO_VALIDATION_REPORT_20260702.md
```

验证数据与图表：

```text
/home/liu/franka_ros2_ws/results/ch3_rcm_comparison_verify_20260702/combined_analysis/analysis
```

本轮已经在线验证以下策略均可在 Gazebo 可视化仿真中完成：

```text
fixed_02
fixed_05
fixed_08
continuous_force_margin
```

当前稳定版本的关键结果：

- RCM 峰值约 `3.52-3.56 mm`，低于 `6.5 mm` 暂停/恢复边界。
- 工具跟踪 RMSE 约 `2.39-2.44 mm`。
- 力跟踪 RMSE 约 `0.047-0.065 N`。
- 姿态误差均值约 `0.72 deg`，front-axis 误差均值约 `0.62 deg`。
- `continuous_force_margin` 的 alpha 在 `0.518-0.820` 之间变化，可用于论文中说明自适应仲裁趋势。

本轮补丁只在 `--local-rcm-task` 下启用 RCM-aware x/y 平移补偿；正式 RCM 控制结构仍保持 tool-to-flange 几何映射和第三章合作博弈力位控制框架。
