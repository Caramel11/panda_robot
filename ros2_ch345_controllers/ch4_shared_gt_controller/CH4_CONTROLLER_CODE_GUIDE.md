# 第四章控制器代码模块说明

本文档说明 `ch4_shared_gt_controller` 中第4章控制器代码的模块划分、核心方法、论文变量对应关系和实验使用方式。

## 1. 总体结构

第4章代码的边界是“共享控制参考层 + 已验证执行层控制器”。也就是说：

- 第4章重点验证人类输入如何进入最终期望轨迹 `x_d`。
- Gazebo、Franka/FR3、Pinocchio 运动学、effort command 接口保持不变。
- 第三章/已有执行层控制器继续负责把 `x_d` 转换为关节力矩。

数据流：

```text
joint_states
  -> Pinocchio frame state
  -> x_r autonomous reference
  -> human delta / x_tele
  -> beta-smoothed x_h
  -> force-safe projection x_h_safe
  -> alpha_HR arbitration
  -> final tool reference x_d
  -> task-space feedback
  -> joint torque command
  -> result.npz / summary.json
```

## 2. 文件职责

| 文件 | 职责 |
| --- | --- |
| `shared_gt_gazebo_node.py` | 第4章在线 Gazebo 控制器主节点，包含参考生成、共享仲裁、安全投影、任务空间控制和数据记录。 |
| `arbitration.py` | 动态人机仲裁权重 `alpha_HR`、sigmoid 基线、0213 风格 GT/LQR 增益。 |
| `pinocchio_model.py` | 机器人模型加载、关节状态扩展、末端/法兰位姿与雅可比计算。 |
| `ch4_metrics.py` | 从 `result.npz` 按论文实验窗口重算指标。 |
| `plot_shared_gt_result.py` | 单次实验时序图绘制。 |
| `compare_ch4_results.py` | 多策略/多场景 CSV 与对比图生成。 |
| `run_ch4_gazebo_suite.py` | 通用批量 Gazebo 启动器。 |
| `run_ch4_comparison_experiments.py` | 固化论文对比实验矩阵的批量运行入口。 |
| `export_ch4_paper_assets.py` | 导出论文用图、表、数据和指标 JSON。 |

## 3. 主控制器模块

主文件：

```text
ch4_shared_gt_controller/shared_gt_gazebo_node.py
```

### 3.1 初始化

类：

```text
SharedGTGazeboNode
```

初始化阶段完成：

- 声明并读取 ROS2 参数。
- 订阅 `/joint_states`。
- 发布 `/no_rcm_effort_controller/commands`。
- 加载 Pinocchio 模型。
- 初始化 `SharedFuzzyArbitrator`。
- 初始化数据记录器 `DataLogger`。

关键参数：

| 参数 | 含义 |
| --- | --- |
| `task_mode` | `no_rcm` 或 `with_rcm`。第4章主实验使用 `no_rcm`。 |
| `controller_variant` | 控制器增益版本，主实验使用 `gt_kf`。 |
| `arbitration_strategy` | 共享控制对比策略。 |
| `scenario` | 人类输入实验场景。 |
| `force_desired` | 期望接触力代理量。 |
| `force_min`, `force_max` | 安全力区间。 |
| `contact_stiffness_hat` | 接触刚度估计，用于参考层力预测。 |
| `task_duration_s` | 单次实验时长，默认 24 s。 |
| `target_tolerance_m` | 稳态误差阈值，理想仿真默认 3 mm；模拟实物实验自动放宽到 10 mm。 |

### 3.2 自主参考 `x_r`

函数：

```text
_nominal_reference(t)
```

作用：

- 生成自主扫描参考 `x_r`。
- 轨迹继承 ROS1 0213 的四点 `a-b-c-d` 结构。
- 前 65% 时间执行运动，后 35% 保持末端目标，用于稳态误差评价。

日志字段：

```text
nominal_ref_0, nominal_ref_1, nominal_ref_2
```

### 3.3 人类输入与遥操作参考 `x_tele`

函数：

```text
_human_delta(t)
```

作用：

- 生成可重复的人类输入 `Delta x_m`。
- 支持外部实时输入场景 `external/falcon_external/live_human`。
- 默认用脚本时间窗复现论文中的人类行为。

场景：

| scenario | 含义 |
| --- | --- |
| `tangential_correction` | 安全切向修正。 |
| `unsafe_normal_push` | 危险法向压入。 |
| `short_pulse_disturbance` | 短时误操作扰动。 |
| `sustained_intervention` | 持续明确干预。 |
| `mixed_sequence` | 四类输入组合。 |

函数：

```text
_candidate_reference(nominal, h_delta)
```

生成：

```text
x_tele = x_r + Delta x_m
x_h = beta x_tele + (1-beta) x_r
```

其中 `beta` 是一阶平滑门控，用于避免小扰动直接进入控制器。

日志字段：

```text
x_tele_0..2
x_h_0..2
beta
```

### 3.4 安全投影 `x_h_safe`

函数：

```text
_predict_force(nominal, candidate)
_safe_project_reference(nominal, human_candidate)
```

力代理模型：

```text
F_hat = F_d + K_hat n_down^T (x_candidate - x_r)
n_down = [0, 0, -1]^T
```

含义：

- 向下法向运动会增加预测接触力。
- 切向运动默认不增加接触力风险，因此尽量保留。
- 法向分量被裁剪到安全力区间对应的位移范围。

输出：

| 变量 | 含义 |
| --- | --- |
| `x_h_safe` | 投影后的安全人侧参考。 |
| `rho_F` | 力安全裕度，越大越安全。 |
| `kappa_N` | 法向保留系数，越小表示越强抑制。 |
| `y_f_candidate` | 投影前预测力。 |
| `y_f_safe` | 投影后预测力。 |
| `normal_raw` | 投影前法向输入。 |
| `normal_safe` | 投影后法向输入。 |

### 3.5 动态仲裁 `alpha_HR`

函数：

```text
_alpha_hr(t, human_delta, tool_pos, safety_distance)
```

调用：

```text
arbitration.py::SharedFuzzyArbitrator.update()
```

输入特征：

| 特征 | 含义 |
| --- | --- |
| `F_h` / `I_h` | 人类输入强度。 |
| `T_h` | 有效输入持续时间。 |
| `D_r` | 安全距离或风险代理量。 |
| `dF_h`, `dT_h`, `dD_r` | 对应变化率。 |

输出：

```text
alpha_HR
alpha_raw
delta_lambda
```

论文解释：

- `alpha_HR -> 0`：偏向机器人自主参考。
- `alpha_HR -> 1`：偏向人类输入。
- 当安全裕度降低时，即使人类输入强，`alpha_HR` 也会被抑制。

### 3.6 最终参考 `x_d`

函数：

```text
_reference(t, tool_pos)
```

完整方法：

```text
x_d = x_r
    + alpha_HR P_T (x_h_safe - x_r)
    + alpha_HR kappa_N P_N (x_h_safe - x_r)
```

其中：

- `P_T` 是切向投影。
- `P_N` 是法向投影。
- `kappa_N` 在接近力边界时降低，进一步抑制危险法向运动。

该函数也是所有对比策略的切换点。

## 4. 对比策略代码

参数：

```text
arbitration_strategy
```

实现位置：

```text
shared_gt_gazebo_node.py::_reference()
```

策略：

| 策略 | 代码行为 | 用途 |
| --- | --- | --- |
| `autonomous_only` | `x_d = x_r` | 无人类输入基线。 |
| `direct_accept` | `x_d = x_tele` | 直接接受遥操作，显示安全风险。 |
| `fixed_blend` | 固定 `alpha_HR` 融合 `x_h` | 固定权重基线。 |
| `single_sigmoid` | 仅由输入幅值决定 `alpha_HR` | 单因素自适应基线。 |
| `dynamic_no_projection` | 动态 `alpha_HR`，但不使用安全投影 | 验证安全投影必要性。 |
| `full_method` | 动态仲裁 + 安全投影 + 切/法向分解 | 论文完整方法。 |

## 5. 执行层控制

函数：

```text
_task_control(tool, flange, t)
```

作用：

- 读取 `_reference()` 返回的最终期望轨迹。
- 根据 `task_mode` 选择 no-RCM 或 with-RCM 控制对象。
- 计算位置误差、速度误差、姿态误差。
- 用 `compute_gt_gain(alpha)` 得到 GT/LQR 形状的反馈增益。
- 混合 Gazebo 稳定 PD 包络。
- 通过雅可比转置映射为关节力矩。

核心公式：

```text
tau = J_v^T u_pos + J_w^T u_rot + tau_g
```

第4章主实验固定执行层，主要比较 `_reference()` 中的共享控制策略。

## 6. 数据记录

类：

```text
DataLogger
```

保存：

```text
result.npz
summary.json
```

重要字段：

| 字段 | 用途 |
| --- | --- |
| `tool_ref_*` | 最终期望参考 `x_d`。 |
| `nominal_ref_*` | 自主参考 `x_r`。 |
| `x_tele_*` | 直接遥操作参考。 |
| `x_h_*` | 人侧候选参考。 |
| `x_h_safe_*` | 投影后安全参考。 |
| `alpha` | 实际使用的 `alpha_HR`。 |
| `alpha_dynamic` | 动态仲裁器输出。 |
| `beta` | 人侧候选参考平滑权重。 |
| `rho_F` | 力安全裕度。 |
| `kappa_N` | 法向保留系数。 |
| `y_f` | 最终参考对应的力代理量。 |
| `tracking_error` | 末端跟踪误差。 |
| `rcm_error` | RCM 误差，仅 with-RCM 有意义。 |

## 7. 指标与绘图

指标脚本：

```text
ch4_metrics.py
```

主要指标：

| 指标 | 含义 |
| --- | --- |
| `tracking_last_rms_m` | 18-24 s 稳态跟踪 RMS；理想仿真要求小于 3 mm，模拟实物实验要求小于 10 mm。 |
| `R_acc` | 安全切向/持续输入接受率。 |
| `R_sup` | 危险法向输入抑制率。 |
| `R_project` | 安全投影层单独贡献。 |
| `T_vio_s` | 力代理量越界总时长。 |
| `F_peak_N` | 相对期望力峰值偏差。 |
| `S_alpha` | `alpha_HR` 变化率 RMS。 |
| `reference_jump_max_m` | 参考轨迹最大相邻跳变。 |

单次绘图：

```bash
ros2 run ch4_shared_gt_controller plot_shared_gt_result --input <run_dir>
```

多策略对比：

```bash
ros2 run ch4_shared_gt_controller compare_ch4_results \
  --runs <run_dir_1> <run_dir_2> \
  --recompute-window-metrics
```

## 8. 推荐阅读顺序

理解控制器时建议按以下顺序读代码：

1. `shared_gt_gazebo_node.py::_reference()`：先理解第4章完整方法和各对比策略。
2. `shared_gt_gazebo_node.py::_human_delta()`：理解每个实验场景输入。
3. `shared_gt_gazebo_node.py::_safe_project_reference()`：理解安全投影。
4. `arbitration.py::SharedFuzzyArbitrator`：理解动态 `alpha_HR`。
5. `shared_gt_gazebo_node.py::_task_control()`：理解参考如何变成力矩。
6. `ch4_metrics.py::compute_metrics()`：理解论文表格指标。
7. `run_ch4_comparison_experiments.py`：理解完整实验矩阵如何执行。
