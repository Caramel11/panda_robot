# 0526 no-RCM 在线力位优先级 + Pareto 迭代控制器说明

本目录是当前可用版本的 no-RCM 控制器复现代码。该版本以
`tests/cooperative_gt_0428/run_no_rcm.py` 的控制流程为基础，在尽量不改变
原有接口的前提下加入两部分内容：

1. `online_priority` 仲裁策略：复现 CAC2026 论文 C. Online Position-Force
   Priority Adaptation，将在线力风险映射为仲裁参数 `alpha`。
2. `pareto_iter` 控制器模式：参考 `tests/0213/0926_circle_200hz.py` 中
   “迭代求解”片段，复现 Algorithm 2 的耦合 Lyapunov/Riccati 迭代，并用于
   no-RCM 力位控制增益求解。

当前主入口是 `run_no_rcm.py`。早期调试失败的独立入口
`run_no_rcm_pareto.py`、独立控制器 `pareto_controller.py`、旧仿真结果和缓存
已删除，避免误用。

## 文件结构

- `run_no_rcm.py`：no-RCM 仿真实验入口，完成接近、接触稳定、恒力扫描、退回。
- `src/alpha_scheduler_gt.py`：所有 alpha 仲裁器，新增
  `OnlinePriorityAdaptationAlphaScheduler`。
- `src/gt_controller.py`：合作博弈力位控制器，新增 `pareto_iter` 模式和
  Algorithm 2 迭代求解。
- `src/robot_interface.py`：机器人状态读取、姿态保持、笛卡尔力到关节力矩映射。
- `src/env_estimator.py`：Kelvin-Voigt 环境刚度/阻尼在线 RLS 估计。
- `src/leaky_integrator.py`：位置/力误差泄漏积分器。
- `src/utils.py`：虚拟接触环境、传感器输入封装、数据记录器。
- `fuzzy_logic.py`、`kalman_filter.py`：保留原有模糊仲裁策略依赖，便于对照实验。

## 控制变量与误差定义

每个笛卡尔轴使用 4 维状态：

```text
z_axis = [e_r1, e_r2, e_f, sigma_f]^T
```

- `e_r1`：位置误差泄漏积分状态，近似累计位置偏差。
- `e_r2`：速度误差。
- `e_f`：力误差，本文采用 `F_actual - F_desired`。
- `sigma_f`：力误差泄漏积分状态。

控制律保持原接口：

```text
u_tool = -K_eff[0] e_r1 - K_eff[1] e_r2 - K_eff[2] e_f - K_eff[3] sigma_f
tau = J_tool^T u_tool + tau_ori
```

其中 `K_eff[0]` 已折入基线虚拟刚度 `K_v`。`alpha` 越大越偏位置跟踪，
`alpha` 越小越偏力调节。

## Online Position-Force Priority Adaptation

新增策略名为：

```bash
--strategy online_priority
```

算法位于 `src/alpha_scheduler_gt.py` 的
`OnlinePriorityAdaptationAlphaScheduler`。核心公式如下：

```text
r_e = |e_f| / eps_f
r_b = max(0,
          (F_norm - F_upper) / (F_max - F_upper),
          (F_lower - F_norm) / (F_lower - F_min))
rho_f = max(r_e, r_b)
alpha_r = alpha_min + (alpha_max-alpha_min) exp(-kappa rho_f^2)
alpha[k+1] = alpha[k] + beta (alpha_r - alpha[k])
beta = clip(lambda_alpha * dt, 0, 1)
```

解释：

- `r_e` 表示归一化力误差风险。
- `r_b` 表示越过安全力边界的风险。
- `rho_f` 取两者较大值，作为当前力风险。
- 风险越大，`alpha_r` 越接近 `alpha_min`，控制器更偏向力调节。
- 自由空间、接近阶段、退回阶段强制 `alpha_r = alpha_max`，保证运动以位置安全为先。

当前默认参数：

```text
alpha_min = 0.05
alpha_max = 0.95
eps_f = 0.25
kappa = 2.5
lambda_alpha = 8.0
F_desired = 1.0 N
F_min = 0.3 N
F_max = 2.0 N
```

## Algorithm 2 Pareto 迭代控制器

启用方式：

```bash
--controller-mode pareto_iter
```

`pareto_iter` 不改变 `compute_control(...)` 的调用接口。它只改变离线增益表的
求解方式：

1. 对每个 `(alpha, K_e)` 网格点构造 4 维轴向系统矩阵 `A, b`。
2. 先用原 ARE 解作为初始稳定策略 `K`。
3. 固定当前策略 `K`，分别求两个目标的 Lyapunov 方程：

   ```text
   A_cl^T P1 + P1 A_cl + Q1 + K^T R1 K = 0
   A_cl^T P2 + P2 A_cl + Q2 + K^T R2 K = 0
   A_cl = A - bK
   ```

4. 使用 Pareto 加权更新策略：

   ```text
   K_new = R_alpha^-1 b^T (alpha P1 + (1-alpha) P2)
   ```

5. 收敛或达到最大迭代次数后保存 `K_eff` 到增益表。
6. 在线阶段仍沿用原双线性插值查表。

注意：调试中发现“只在 `alpha0=0.5` 求 `P1/P2`，再用在线 alpha 外推”的版本
会导致接近阶段错误混入力目标，工具端接触后离开表面并 timeout。因此当前版本
改为每个 `(alpha, K_e)` 网格点单独迭代，已通过仿真验证。

## 使用指南

先启动 Gazebo：

```bash
cd /home/liu/franka_ws_1101 #替换为实际工作空间目录
source devel/setup.bash
roslaunch panda_gazebo panda_world.launch use_custom_action_servers:=false
rosrun panda_sim_custom_action_server start_joint_trajectory_server.py
rosrun panda_sim_custom_action_server start_gripper_action_server.py
roslaunch panda_sim_moveit sim_move_group.launch
roslaunch panda_simulator_examples demo_moveit.launch
```

推荐运行当前版本：

```bash
cd /home/liu/franka_ws_1101 #替换为实际工作空间目录
source devel/setup.bash
python3 src/panda_robot/tests/0526_controller/run_no_rcm.py \
  --strategy online_priority \
  --controller-mode pareto_iter \
  --trials 1
```

未指定 `--output-dir` 时，结果默认保存到 `/home/liu/franka_ws_1101/results`，
与绘图脚本的默认搜索目录一致。

只验证内容 1，即在线 alpha 仲裁但使用原 ARE 控制器：

```bash
python3 src/panda_robot/tests/0526_controller/run_no_rcm.py \
  --strategy online_priority \
  --controller-mode are \
  --trials 1
```

复用已经保存的增益表：

```bash
python3 src/panda_robot/tests/0526_controller/run_no_rcm.py \
  --strategy online_priority \
  --controller-mode pareto_iter \
  --gains-file /home/liu/franka_ws_1101/results/coop_gains_no_rcm_pareto_iter.npy
```

## 输出数据

每次实验会在 `--output-dir/no_rcm_YYYYmmdd_HHMMSS/` 下保存 `.npz` 文件。
常用字段：

- `t`：扫描阶段时间。
- `pos_x/y/z`、`pos_des_x/y/z`：实际/期望 tool 位置。
- `pos_err_norm`：位置误差范数。
- `F_measured`、`F_desired`、`F_err`：接触力、目标力和误差。
- `alpha`：在线仲裁参数。
- `K_total`、`K_r2`、`K_ef`、`K_sf`：当前插值后的控制增益。
- `K_hat`：RLS 估计的环境刚度。
- `phase`：alpha 调度器识别的阶段。

## 绘图与 Excel 分析

本目录提供 `plot_experiment_analysis.py`，用于默认分析最近一次实验结果：

```bash
cd /home/liu/franka_ws_1101/src/panda_robot/tests/0526_controller
python3 plot_experiment_analysis.py
```

脚本默认搜索：

```text
/home/liu/franka_ws_1101/results/no_rcm_*/*.npz
```

无图形界面或只想保存文件：

```bash
python3 plot_experiment_analysis.py --no-show
```

指定某个 `.npz`：

```bash
python3 plot_experiment_analysis.py \
  --input /home/liu/franka_ws_1101/results/no_rcm_YYYYmmdd_HHMMSS/continuous_force_margin_alpha_t00.npz
```

输出会保存在对应实验目录的 `analysis/` 子目录中：

- `01_force_alpha.png/.pdf`：力跟踪、力误差、alpha。
- `02_position_tracking.png/.pdf`：三轴位置跟踪和位置误差。
- `03_gain_estimation.png/.pdf`：环境刚度估计和控制增益。
- `04_control_state_phase.png/.pdf`：控制量、误差状态和阶段。
- `05_distribution_scatter.png/.pdf`：误差分布与相关性散点图。
- `online_priority_alpha_t00_analysis.xlsx`：Excel 表格，含 summary、phase、force band 和 time series。

## 已验证结果

在 Gazebo 中使用：

```bash
--strategy online_priority --controller-mode pareto_iter --trials 1
```

完成 4000 个扫描采样。该次结果指标为：

- 力 RMSE：约 `0.0496 N`
- 力范围：约 `0.656-1.069 N`
- 最终力：约 `1.030 N`
- 位置误差均值：约 `1.33 mm`
- 最大位置误差：约 `2.12 mm`
- 最终位置误差：约 `0.76 mm`
- alpha 范围：约 `0.181-0.950`

这些结果说明修复后的 `pareto_iter` 模式可以完成接触、扫描和退回流程。

## 调试建议

- 若接触后工具端离开表面，优先检查 `alpha=1` 附近的 Pareto 增益是否混入了力目标。
- 若仿真长时间卡在接近阶段，检查 `F_measured` 是否持续为 0，以及 `scan_z` 与
  `approach_z` 是否合理。
- 若 `--gains-file` 载入后行为异常，删除旧增益文件重新预计算。
- 若只想隔离 alpha 问题，先用 `--controller-mode are`；若只想隔离控制器问题，
  可对比 `fixed_05` 或 `fixed_08` 策略。
