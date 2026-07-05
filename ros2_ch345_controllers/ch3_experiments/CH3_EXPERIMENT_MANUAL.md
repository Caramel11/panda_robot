# 第三章实验代码使用与论文对应手册

生成日期：2026-07-02

本文档说明 `/home/liu/franka_ros2_ws/src/ch3_experiments` 如何支撑第三章实验。该包采用“最小改动”策略：第三章实验组织、仿真基线、指标统计和报告生成放在新包中；已经稳定的 ROS2/Gazebo 在线控制仍由 `src/no_rcm` 执行。

## 1. 方法边界

第三章只研究执行层力位协同控制。输入是给定的扫描轨迹、期望法向接触力和柔性环境；输出是任务空间控制量或 Gazebo 中的关节力矩命令。第三章不引入操作者输入、不使用第四章人机仲裁权重 `alpha_HR`。

第三章核心变量是执行层力位优先级：

```text
alpha_FP = 1   更偏位置/轨迹跟踪
alpha_FP = 0   更偏力跟踪/力安全
```

在 no-RCM Gazebo 控制器中，实际采用 z-only 仲裁：

```text
alpha_x = 1
alpha_y = 1
alpha_z = alpha_FP
```

含义是 x/y 切向扫描保持位置优先，z 法向根据力误差、刚度和力安全裕度调节力位折中。

## 2. 数学模型

柔性接触环境采用分段 Kelvin-Voigt 模型：

```text
F_n = K_e(x) delta + B_e(x) max(0, -dot z)
delta = max(0, z_surface - z_tool)
```

其中 `K_e(x)` 与 `B_e(x)` 随扫描位置分段变化，用于模拟软区、刚度切换区和硬区。

第三章合作博弈控制器使用增广状态：

```text
xi = [e_p, e_v, e_f, sigma_f]^T
e_p = x - x_d
e_v = dx - dx_d
e_f = F - F_d
dot sigma_f = e_f - epsilon_f sigma_f
```

位置目标和力目标的代价函数分别为：

```text
J_p = integral (e_p^T Q_p e_p + e_v^T Q_v e_v + u^T R_p u) dt
J_f = integral (e_f^T Q_f e_f + sigma_f^T Q_sigma sigma_f + u^T R_f u) dt
```

执行层合作博弈折中为：

```text
J(alpha_FP) = alpha_FP J_p + (1 - alpha_FP) J_f
```

`src/no_rcm` 在线控制器通过 ARE/Pareto 迭代预计算增益表，在线根据 `alpha_FP` 与 `K_hat` 插值。`ch3_experiments` 纯 Python 仿真为了批量对比和快速出图，使用同一变量语义的轻量近似控制器；Gazebo 在线验证仍调用 `no_rcm` 的真实控制器。

## 3. 代码与论文实验对应关系

| 论文实验内容 | 代码入口 | 输出 |
| --- | --- | --- |
| 柔性环境分段刚度建模 | `flexible_surface.py` | `K_env_true`, `B_env_true`, `stiffness_zone_label` |
| 固定扫描轨迹 | `reference_trajectory.py` | `pos_des_*`, `scan_vx_eff` |
| 阻抗基线 | `controllers/impedance.py` | `impedance_t*.npz` |
| 经典混合力位基线 | `controllers/hybrid_force_position.py` | `hybrid_t*.npz` |
| 固定优先级合作博弈 | `controllers/fixed_gt.py` | `fixed_02/fixed_05/fixed_08` |
| 动态优先级合作博弈 | `controllers/dynamic_gt.py`, `alpha_fp_analysis.py` | `dynamic_gt_t*.npz` |
| 受限 QP/MPC 风格基线 | `controllers/mpc_qp.py` | `mpc_qp_t*.npz` |
| 纯 Python 对比实验 | `run_ch3_sim.py` | `.npz`, `.csv`, `.png`, `.md` |
| Gazebo/no-RCM 在线验证 | `run_ch3_gazebo_suite.py` | Gazebo/no-RCM 在线数据与报告 |
| Gazebo/RCM 在线验证 | `run_ch3_rcm_gazebo_suite.py`, `analyze_ch3_rcm_result.py` | RCM 误差、工具跟踪、力跟踪与 alpha 报告 |
| 第三章指标统计 | `ch3_metrics.py` 调用 `no_rcm.analyze_ch3_result` | `ch3_strategy_summary.csv` |

## 4. 对比实验设计

建议第三章至少包含三组实验。

### 4.1 纯 Python 分段刚度对比

目的：快速验证不同控制器在软区、硬区和刚度切换处的力位折中。

命令：

```bash
source /opt/ros/humble/setup.bash
source /home/liu/franka_ros2_ws/install/setup.bash

ros2 run ch3_experiments run_ch3_sim \
  --controllers standard_impedance,traditional_hybrid,standard_mpc,fixed_02,fixed_05,fixed_08,dynamic_gt \
  --repeats 5 \
  --output-dir /home/liu/franka_ros2_ws/results/ch3_experiments_sim
```

论文应使用的指标：

```text
force_rmse_N
force_peak_abs_err_N
force_jitter_N
force_violation_time_s
tangent_pos_rmse_mm
normal_pos_rmse_mm
alpha_mean / alpha_min / alpha_max
alpha_khat_corr
compute_time_avg_ms
```

期望结果：

- `standard_impedance` 是标准阻抗/位置优先基线，位置误差通常较小，但不显式闭合法向力环，力误差和越界风险更高。
- `traditional_hybrid` 是传统混合力位控制，切向跟踪位置、法向闭合力误差，力跟踪较直接，但刚度切换处可能产生抖动或法向位置误差。
- `standard_mpc` 是短预测窗 MPC/QP 风格基线，通过力边界约束修正法向输入，预期越界风险低于阻抗，但权重变化不如本文动态 GT 连续可解释。
- `fixed_02` 更偏力，位置误差较大。
- `fixed_08` 更偏位置，硬区力误差和安全风险更高。
- `fixed_05` 是折中基线。
- `dynamic_gt` 的 `alpha_FP` 应随刚度和力安全裕度变化，在硬区或安全裕度充足时提高，在接近力边界时降低。
- `mpc_qp` 应体现输入/安全约束的参考价值，但不作为本文主方法。

### 4.1.1 多控制器结构对照补充实验

目的：解决第三章“仅展示本文控制器、缺少其他控制器对照”的问题。该实验在相同柔性接触模型、相同期望接触力和相同扫描轨迹下，统一比较以下控制器：

| 对照方法 | 代码位置 | 对照意义 |
| --- | --- | --- |
| 标准阻抗控制 | `controllers/impedance.py` / `execution_strategy:=standard_impedance` | 说明无显式力位仲裁时的力安全代价 |
| 传统混合力位控制 | `controllers/hybrid_force_position.py` / `execution_strategy:=traditional_hybrid` | 作为经典切向位置、法向力控制基线 |
| 标准 MPC/QP 控制 | `controllers/mpc_qp.py` / `execution_strategy:=standard_mpc` | 作为预测约束类方法基线，观察约束投影和本文连续仲裁的差异 |
| 固定优先级合作博弈 | `controllers/fixed_gt.py` | 通过 `fixed_02/fixed_05/fixed_08` 展示固定 alpha 的权衡规律 |
| 动态优先级合作博弈 | `controllers/dynamic_gt.py` | 本文方法，展示 `alpha_FP` 随力安全裕度和刚度变化的适应性 |
| MPC/QP 风格约束基线 | `controllers/mpc_qp.py` | 对应历史 ROS1 GT 与 MPC 对照思路，用于提供约束控制参考 |

ROS1 历史参考中，`panda_robot_gt_controller_dev/tests/0213` 下的 `real_GT_KF.py`、`real_GT_Sigmoid.py`、`real_MPC_KF.py` 和 `real_MPC_Sigmoid.py` 已经提供了 GT/MPC、KF/Sigmoid 的四方法对照逻辑。当前 ROS2 第三章执行层不引入第四章人机输入，因此将其收敛为阻抗、混合力位、固定 GT、动态 GT 和 MPC/QP 风格基线的结构对照。

推荐命令：

```bash
source /opt/ros/humble/setup.bash
source /home/liu/franka_ros2_ws/install/setup.bash

ros2 run ch3_experiments run_ch3_controller_baseline_comparison \
  --controllers standard_impedance,traditional_hybrid,standard_mpc,fixed_02,fixed_05,fixed_08,dynamic_gt \
  --repeats 3 \
  --output-dir /home/liu/franka_ros2_ws/results/ch3_controller_baseline_comparison
```

输出包括：

```text
CH3_CONTROLLER_BASELINE_COMPARISON_REPORT.md
figures/ch3_controller_baseline_metric_bars_cn.png
figures/ch3_controller_baseline_zone_bars_cn.png
figures/ch3_controller_baseline_timeseries_cn.png
ch3_analysis/analysis/ch3_strategy_summary.csv
ch3_analysis/analysis/ch3_trial_metrics.csv
```

论文写作建议：该实验应作为第三章“控制器对比实验”的基础结果。标准阻抗控制用于说明只重视位置会放大力安全风险；传统混合力位控制用于说明固定方向解耦缺少连续折中；标准 MPC/QP 用作预测约束控制参考；固定 GT 用作 `alpha_FP` 消融；动态 GT 应重点解释其在力安全、位置误差和刚度切换适应性之间的综合折中。

三类新增标准控制器的原理差异与预期趋势：

- 标准阻抗控制：三轴弹簧阻尼模型，输入由位置误差和速度误差决定。预期几何轨迹跟踪较好，但接触力只被动响应环境，力峰值和越界时间可能增大。
- 传统混合力位控制：切向方向使用位置控制，法向方向使用力误差反馈。预期法向力误差较小，但法向位置误差、曲线轨迹一致性和刚度切换平滑性可能变差。
- 标准 MPC/QP 控制：用短预测窗检查法向力边界，并在可能越界时修正法向输入。预期能降低越界风险，但控制行为主要在约束激活时发生，不具备本文 `alpha_FP` 随力裕度、刚度和任务阶段连续变化的解释性。
- 本文动态 GT：通过合作博弈代价和力安全裕度调节 `alpha_FP`，在位置目标和力目标之间连续折中。预期不必在单项力误差或单项位置误差上绝对最优，但综合误差、越界时间和阶段适应性更稳定。

### 4.2 Gazebo/no-RCM 在线稳定性验证

目的：确认 ROS2/Gazebo 中接近、扫描、复位可以稳定完成，且末端姿态不抖动、不发散。

命令：

```bash
source /opt/ros/humble/setup.bash
source /home/liu/franka_ros2_ws/install/setup.bash

ros2 run ch3_experiments run_ch3_gazebo_suite \
  --strategies fixed_05 \
  --trials 1 \
  --output-dir /home/liu/franka_ros2_ws/results/ch3_experiments_gazebo
```

如需显示 Gazebo GUI：

```bash
ros2 run ch3_experiments run_ch3_gazebo_suite \
  --strategies fixed_05 \
  --trials 1 \
  --gui
```

期望结果：

- `no_rcm_effort_controller` active。
- 接近阶段能稳定接触。
- 扫描阶段接触力在安全边界内。
- 复位阶段回到初始笛卡尔位置附近。
- 末端姿态误差建议小于 `3 deg`，正面朝向误差建议小于 `3 deg`。
- 在线计算时间均值应远低于控制周期。

### 4.3 Gazebo/no-RCM 策略对比

目的：在真实 ROS2 控制器路径下比较固定优先级和动态优先级。

命令：

```bash
ros2 run ch3_experiments run_ch3_gazebo_suite \
  --strategies fixed_02,fixed_05,fixed_08,continuous_force_margin \
  --trials 1 \
  --output-dir /home/liu/franka_ros2_ws/results/ch3_experiments_gazebo_compare
```

该实验耗时较长。若出现 Gazebo 资源占用或控制器未 active，应先只跑 `fixed_05` 做健康检查，再扩大策略组。

### 4.4 刚度和噪声不确定性扫描

目的：验证第三章控制器不是只在单一标称刚度和无噪声条件下有效。该实验只运行纯 Python 仿真，适合论文鲁棒性图表和参数敏感性表格。

命令：

```bash
source /opt/ros/humble/setup.bash
source /home/liu/franka_ros2_ws/install/setup.bash

ros2 run ch3_experiments run_ch3_uncertainty \
  --controllers fixed_05,dynamic_gt \
  --repeats 3 \
  --stiffness-scales 0.75,1.0,1.25 \
  --force-noise-stds 0.0,0.004,0.012 \
  --output-dir /home/liu/franka_ros2_ws/results/ch3_experiments_uncertainty
```

输出包括每个参数组合的 `ch3_strategy_summary.csv`，以及总表 `ch3_uncertainty_manifest.csv` 和 `CH3_UNCERTAINTY_SWEEP_REPORT.md`。

### 4.5 Gazebo/RCM 约束验证

目的：验证第三章执行层力位协同控制器在带穿刺点几何约束时仍能稳定完成接近、接触调整和扫描。该实验重点观察 RCM 误差、工具跟踪误差、力跟踪误差和 `alpha_FP` 随环境变化的响应。

命令：

```bash
source /opt/ros/humble/setup.bash
source /home/liu/franka_ros2_ws/install/setup.bash

ros2 run ch3_experiments run_ch3_rcm_gazebo_suite \
  --strategies fixed_05 \
  --trials 1 \
  --controller-mode pareto_iter \
  --output-dir /home/liu/franka_ros2_ws/results/ch3_experiments_rcm_gazebo
```

扩展对比：

```bash
ros2 run ch3_experiments run_ch3_rcm_gazebo_suite \
  --strategies fixed_02,fixed_05,fixed_08,continuous_force_margin \
  --trials 1
```

期望结果：

- `no_rcm_effort_controller` active。该 controller 仅作为 7 关节 effort 后端，RCM 几何约束仍由 `run_with_rcm` 完成。
- 接近阶段 RCM 误差不超过软阈值太久。
- 扫描阶段 `rcm_rmse_mm` 保持在毫米级，峰值不触发硬中止。
- 力跟踪误差和位置跟踪误差有界收敛。
- 当 RCM 误差或接触力风险上升时，动态策略应提高位置/RCM 优先级并降低推进速度。

## 5. 输出文件

每次纯 Python 仿真会生成：

```text
results/ch3_experiments_sim/ch3_sim_YYYYMMDD_HHMMSS/
  impedance_t00.npz
  hybrid_t00.npz
  fixed_0.5_t00.npz
  dynamic_gt_t00.npz
  ch3_analysis/analysis/ch3_trial_metrics.csv
  ch3_analysis/analysis/ch3_strategy_summary.csv
  ch3_analysis/analysis/ch3_controller_metric_bars.png
  ch3_analysis/analysis/ch3_representative_timeseries.png
  ch3_analysis/analysis/ch3_force_position_analysis_report.md
```

Gazebo 在线实验会生成：

```text
results/ch3_experiments_gazebo/no_rcm_YYYYMMDD_HHMMSS/
  fixed_0.5_t00.npz
  ch3_analysis/analysis/*.csv
  ch3_analysis/analysis/*.png
  ch3_analysis/analysis/ch3_force_position_analysis_report.md
```

RCM 在线实验会生成：

```text
results/ch3_experiments_rcm_gazebo/rcm_YYYYMMDD_HHMMSS/
  fixed_0.5_t00.npz
  approach_debug/approach_fixed_0.5_t00.npz
  ch3_rcm_analysis/analysis/ch3_rcm_trial_metrics.csv
  ch3_rcm_analysis/analysis/ch3_rcm_strategy_summary.csv
  ch3_rcm_analysis/analysis/ch3_rcm_metric_bars.png
  ch3_rcm_analysis/analysis/ch3_rcm_representative_timeseries.png
  ch3_rcm_analysis/analysis/ch3_rcm_analysis_report.md
```

## 6. 稳定性注意事项

- 不要在第三章实验包中直接改 `no_rcm/run_no_rcm.py` 的接近、扫描、复位稳定逻辑。
- 新的 `alpha_FP` 策略应先在纯 Python 仿真验证，再接入 `no_rcm`。
- `alpha_FP` 必须限幅并低通滤波，避免刚度区切换时控制量突变。
- 力安全边界附近应降低位置优先级，优先减小法向力风险。
- Gazebo 在线验证建议先运行单策略、单试次，确认稳定后再扩大对比实验。
- 第三章结果统计只评价执行层，不应把第四章人机参考仲裁指标混入。
- RCM 实验中 `alpha_FP` 的解释需要同时考虑力跟踪和穿刺点几何安全；RCM 误差升高时，控制器提高位置/几何优先级是符合预期的。

## 7. 当前完成状态

已实现：

- ROS2/Python 包骨架。
- 纯 Python 柔性接触环境与固定扫描参考。
- 五类第三章对比控制器。
- 统一 `.npz` 日志字段。
- 复用 `no_rcm.analyze_ch3_result` 的指标、绘图和 `.md` 报告。
- Gazebo/no-RCM 在线实验编排入口。
- Gazebo/RCM 在线实验编排、指标统计、绘图和报告入口。
- 刚度倍率和力噪声不确定性扫描入口。

当前推荐先采用 `fixed_05` 做 Gazebo 健康检查，再运行 `continuous_force_margin` 与固定优先级组做论文对比。

论文实验撰写、数据准备和代码对应细节见 `CH3_THESIS_EXPERIMENT_WRITING_GUIDE.md`。
