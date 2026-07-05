# 第三章实验撰写、数据准备与代码对应指南

生成日期：2026-07-02

本文档用于把论文第 3 章正文、实验数据和 `/home/liu/franka_ros2_ws/src/ch3_experiments` 代码一一对应。第 3 章定位为执行层合作博弈力--位协同控制，不讨论第 4 章的人机参考仲裁。

## 1. 第三章实验部分建议结构

建议第 3 章实验小节按以下顺序撰写。

### 1.1 实验目的

需要说明本章实验验证四件事：

1. 合作博弈力--位控制器能同时处理位置目标和接触力目标。
2. `alpha_FP` 是执行层力位优先级，不是人机权限；它能解释控制器在位置跟踪和力安全之间的折中。
3. 动态 `alpha_FP` 相比固定优先级能在刚度变化和力安全边界附近给出更合理的调节。
4. ROS2/Gazebo no-RCM 在线控制器可稳定完成接近、扫描和复位，为后续 RCM 和实物实验提供执行层基线。

正文中不要写本地路径、命令或软件调试细节。路径和命令只放在实验附录或代码说明文档中。

### 1.2 实验对象与任务

应描述为：

```text
末端工具与分段柔性材料保持接触，并沿给定切向轨迹扫描。
控制器需要跟踪切向位置，同时维持法向接触力在期望值附近和安全力边界内。
```

应给出参数表：

| 参数 | 建议内容 |
| --- | --- |
| 期望力 | `F_d = 1.0 N` |
| 安全力范围 | 例如 `[0.3, 2.0] N` 或仿真中的 `[0.66, 1.38] N` |
| 扫描区间 | `x = 0.40 m` 到 `0.48 m` |
| 柔性环境 | 软区、硬区刚度和阻尼 |
| 控制周期 | `dt = 0.01 s` |
| 优先级范围 | `alpha_FP` 的上下限 |

### 1.3 对比方法

建议至少写 5 类：

| 方法 | 论文含义 | 代码 |
| --- | --- | --- |
| 固定阻抗 | 只强调位置/阻抗，不显式调节力安全 | `controllers/impedance.py` |
| 混合力位控制 | 切向位置 + 法向力 PI/PD | `controllers/hybrid_force_position.py` |
| 固定优先级 GT | 固定 `alpha_FP` 的合作博弈控制 | `fixed_02`, `fixed_05`, `fixed_08` |
| 动态优先级 GT | 由力裕度和刚度估计调节 `alpha_FP` | `dynamic_gt` / Gazebo 中 `continuous_force_margin` |
| QP/MPC 风格约束基线 | 输入和力边界约束参考 | `controllers/mpc_qp.py` |

若正文篇幅有限，主图中可放 4 类：阻抗、混合力位、固定 GT、动态 GT；QP/MPC 可放补充表。

### 1.4 评价指标

必须准备以下指标：

```text
力 RMSE
峰值力误差
力抖动
力越界时间
切向位置 RMSE
法向位置 RMSE
整体位置 RMSE
alpha_FP 均值/范围
alpha_FP 与 K_hat 的相关性
单周期计算时间
Gazebo 姿态误差和正面朝向误差
复位最终位置误差
```

## 2. 需要进行哪些实验

### 实验 A：标称分段刚度仿真对比

目的：证明合作博弈控制器比单一目标控制器更适合柔性接触力位冲突。

代码：

```bash
ros2 run ch3_experiments run_ch3_sim \
  --controllers impedance,hybrid,fixed_02,fixed_05,fixed_08,dynamic_gt,mpc_qp \
  --repeats 5 \
  --output-dir /home/liu/franka_ros2_ws/results/ch3_experiments_sim
```

论文图：

- 接触力时序图：`F_n`、`F_d`、`F_min`、`F_max`。
- 位置误差时序图：切向和法向误差。
- 指标箱线图或柱状图：力 RMSE、越界时间、位置 RMSE、力抖动。

期望结论：

- 固定阻抗位置稳定但力误差较大。
- 混合力位力跟踪直接但刚度切换处可能抖动。
- 固定 GT 能体现不同 `alpha_FP` 的折中。
- 动态 GT 在力安全和位置误差之间有更好的综合表现。

### 实验 B：`alpha_FP` 机理实验

目的：解释动态 `alpha_FP` 为什么变化，变化是否符合物理直觉。

数据字段：

```text
alpha_z
K_hat
K_env_true
rho_F
s_F
F_err
stiffness_zone_label
```

论文图：

- `alpha_FP` 与 `K_hat/K_env` 同图。
- `alpha_FP` 与力安全裕度 `rho_F` 同图。
- 软区、刚度切换区、硬区的分区指标表。

期望结论：

- 在刚度升高且力裕度充足时，`alpha_FP` 可提高位置优先级。
- 在接近力边界或力误差变大时，`alpha_FP` 应降低，给力调节更多权重。
- `alpha_FP` 变化应连续，不能产生跳变型控制抖动。

### 实验 C：噪声和刚度不确定性鲁棒性

目的：证明控制器不是只在单一标称参数下有效。

代码：

```bash
ros2 run ch3_experiments run_ch3_uncertainty \
  --controllers fixed_05,dynamic_gt \
  --repeats 3 \
  --stiffness-scales 0.75,1.0,1.25 \
  --force-noise-stds 0.0,0.004,0.012 \
  --output-dir /home/liu/franka_ros2_ws/results/ch3_experiments_uncertainty
```

论文图：

- 不同刚度倍率下的力 RMSE 和位置 RMSE。
- 不同噪声强度下的力抖动和越界时间。
- 动态 GT 与固定 GT 的鲁棒性对比表。

期望结论：

- 动态 GT 的力越界时间和力抖动随噪声增长应更平缓。
- 刚度变化时，动态 GT 的 `alpha_FP` 应有可解释变化。

### 实验 D：Gazebo/no-RCM 在线可视化验证

目的：证明 ROS2 控制器在机器人动力学、Pinocchio、Gazebo effort controller 和可视化环境下可稳定运行。

代码：

```bash
ros2 run ch3_experiments run_ch3_gazebo_suite \
  --strategies fixed_05 \
  --trials 1 \
  --gui \
  --output-dir /home/liu/franka_ros2_ws/results/ch3_experiments_gazebo_gui_verify
```

论文图：

- Gazebo 在线接触力和位置误差时序。
- 末端姿态误差和正面朝向误差。
- 复位阶段最终位置/姿态误差表。

期望结论：

- 接近、扫描、复位均稳定。
- 力越界时间为 0 或接近 0。
- 末端姿态误差保持在小角度范围。
- 在线计算时间显著小于控制周期。

### 实验 E：Gazebo 固定优先级与动态优先级对比

目的：把纯仿真的动态 `alpha_FP` 优势迁移到真实 ROS2/Gazebo 控制路径中。

代码：

```bash
ros2 run ch3_experiments run_ch3_gazebo_suite \
  --strategies fixed_02,fixed_05,fixed_08,continuous_force_margin \
  --trials 1 \
  --gui \
  --output-dir /home/liu/franka_ros2_ws/results/ch3_experiments_gazebo_compare
```

该实验耗时较长，建议先确保实验 D 成功。

## 3. 已有代码能完成什么

当前已经具备：

- `run_ch3_sim`：标称纯 Python 多控制器对比。
- `run_ch3_uncertainty`：刚度倍率和力噪声扫描。
- `run_ch3_gazebo_suite`：调用稳定 `no_rcm` 的 Gazebo 在线实验。
- `no_rcm.analyze_ch3_result`：统一生成 `.csv`、`.png` 和 `.md`。
- `CH3_EXPERIMENT_MANUAL.md`：方法、运行命令和注意事项。
- `CH3_GAZEBO_EXPERIMENT_REPORT.md`：当前 Gazebo 验证结果。

## 4. 仍建议补充的代码

优先级从高到低：

1. RCM 条件下的执行层力位控制实验包或适配器，用于证明方法不依赖 no-RCM 场景。
2. 更接近真实实验的数据采集入口，包括真实力传感器零偏、工具坐标标定、材料刚度标定和安全停止。
3. 论文专用绘图脚本，把现有分析图转换为统一字体、线宽、尺寸和中文/英文标签。
4. 多试次统计显著性脚本，例如 Wilcoxon 或配对 t 检验，用于支撑“显著改善”而不是只给均值。
5. `continuous_force_margin` 在 Gazebo 多策略对比中的批量运行和异常自动恢复。

其中第 1 项和第 2 项涉及新的硬件/几何约束，不能用当前 no-RCM 包直接替代。第 3 至第 5 项可以在 `ch3_experiments` 内继续增量实现。

## 5. 写作时的关键注意事项

- 第 3 章只写 `alpha_FP`，不要写成人机权限。
- 第 4 章才写 `alpha_HR`，它只作用于参考生成。
- 不要把 Gazebo 单次验证数值写成普遍结论；正式论文应使用多试次统计。
- 不要声称力安全是严格硬约束；当前证据支持的是实验范围内越界时间小和控制量有界。
- 若动态 GT 指标没有全面优于所有基线，应写成“综合折中更稳定”，不要写成“所有指标最优”。
