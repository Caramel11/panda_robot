# 0608controller 第五章双仲裁仿真方法与使用说明

`tests/0608controller` 用于生成论文第五章“参考层人机仲裁 + 执行层力位仲裁”的可复现实验。脚本 `run_ch5_dual_arbitration.py` 是自包含的确定性仿真入口，同时带有一个轻量 Gazebo/ROS 探测接口，用于记录本地仿真环境是否可达。

## 1. 方法目标

第五章实验面向工业约束接触任务。机器人沿工件表面扫描，操作者可以给出切向修正，也可能误给法向压入。控制器需要同时满足三类目标：

1. 接受安全的人类切向修正，使轨迹绕开缺陷或覆盖重点区域。
2. 抑制危险的人类法向压入，避免接触力越界。
3. 在变刚度环境中动态调节力-位优先级，兼顾轨迹误差和力误差。

代码中使用两个仲裁参数：

- `alpha_hr`：参考层人机仲裁参数，只影响期望轨迹 `x_d` 的生成。
- `alpha_fp`：执行层力位仲裁参数，只影响 z 向压入深度和接触力调节。

这种分层设计避免把人的输入、接触安全和力位控制混到同一个权重里。切向输入主要经过 `alpha_hr` 进入参考轨迹；法向输入先经过安全投影，再由 `rho_F` 和 `alpha_fp` 决定是否允许进入执行层。

## 2. 核心模型

环境刚度由 `environment_stiffness(x)` 分段定义：

```text
x < 0.026             K = 320 N/m
0.026 <= x < 0.052    K = 860 N/m
0.052 <= x < 0.078    K = 230 N/m
x >= 0.078            K = 1120 N/m
```

名义参考 `nominal_reference(t)` 生成一条 x 方向扫描线，`human_delta(t)` 叠加操作者输入：

- 10-20 s：正 y 方向安全切向修正。
- 24-30.5 s：负 z 方向不安全法向压入。
- 32-39 s：反向切向修正。
- 40-41.2 s：短时法向扰动。

接触力按局部 Kelvin-Voigt 近似计算：

```text
F = K_true * max(0, surface_z - z) + B_true * max(0, -z_dot) + noise
```

力安全裕度 `rho_F` 由 `F_min`、`F_desired`、`F_max` 给出。`rho_F` 越小，说明当前力越接近上下边界，执行层会更偏向力调节。

## 3. 对比方法

默认比较六种方法：

- `fixed_08`：固定 `alpha_fp = 0.8`，偏位置。
- `fixed_05`：固定 `alpha_fp = 0.5`，折中。
- `fixed_02`：固定 `alpha_fp = 0.2`，偏力。
- `hr_only`：只有参考层人机动态仲裁，执行层固定。
- `fp_only`：只有执行层力位动态仲裁，不接收人类修正。
- `dual_arbitration`：同时启用参考层和执行层动态仲裁。

## 4. 运行命令

推荐从工作空间根目录运行：

```bash
cd /home/liu/franka_ws_1101
MPLCONFIGDIR=/tmp/matplotlib-codex \
python3 src/panda_robot/tests/0608controller/run_ch5_dual_arbitration.py \
  --backend analytic \
  --tune \
  --seed 11
```

如果希望记录本地 Gazebo/ROS 可达性：

```bash
cd /home/liu/franka_ws_1101
source devel/setup.bash
MPLCONFIGDIR=/tmp/matplotlib-codex \
python3 src/panda_robot/tests/0608controller/run_ch5_dual_arbitration.py \
  --backend gazebo \
  --tune \
  --seed 11
```

`--backend gazebo` 不会强行杀掉用户已有 Gazebo 进程，只会探测 ROS master、topic 和模型状态，并把诊断写入结果目录。

## 5. 输出文件

每次运行生成：

```text
tests/0608controller/results/ch5_dual_arbitration_YYYYMMDD_HHMMSS/
  metrics.csv
  tuning_history.csv
  fixed_08.npz
  fixed_05.npz
  fixed_02.npz
  hr_only.npz
  fp_only.npz
  dual_arbitration.npz
  metrics_bars.png
  force_alpha_stiffness_timeseries.png
  trajectory_xy.png
  unsafe_normal_push_detail.png
  dual_alpha_khat_response.png
  gazebo_status.json
  summary.md
```

图像含义：

- `metrics_bars.png`：力 RMSE、任务跟踪 RMSE、越界时间和综合评分。
- `force_alpha_stiffness_timeseries.png`：接触力、`alpha_fp` 和估计刚度时序。
- `trajectory_xy.png`：切向修正和缺陷绕行轨迹。
- `unsafe_normal_push_detail.png`：不安全法向压入时的力、z 和 `alpha_fp` 响应。
- `dual_alpha_khat_response.png`：`alpha_fp` 随估计刚度和力安全裕度变化的散点图。

## 6. 评价指标

`metrics.csv` 中的关键列：

- `score`：综合评分，越小越好。
- `force_rms_N`：接触力误差均方根。
- `force_peak_N`：最大力误差。
- `force_jitter_N`：力抖动。
- `upper_violation_s`、`lower_violation_s`：超过力上下界的时间。
- `task_rms_mm`：相对安全任务参考的跟踪误差。
- `tangent_acceptance`：安全切向输入接受率。
- `normal_suppression`：不安全法向输入抑制率。
- `alpha_fp_min/max`：执行层仲裁参数变化范围。
- `alpha_fp_khat_corr`：`alpha_fp` 与估计刚度的相关性。

## 7. 调参与复现实验

自动调参由 `tune_config()` 完成，扫描：

- `dual_stiffness_blend`
- `dual_force_gain`
- `dual_risk_gain`
- `dual_alpha_soft`
- `dual_alpha_hard`

如果要凸显动态仲裁相对固定参数的优势，可以从以下方向调节：

- 增大分段刚度差异，让固定 `alpha` 无法同时适配软硬区域。
- 缩小 `F_min/F_max`，使力安全裕度对策略更敏感。
- 提高不安全法向输入幅值，检验 `normal_suppression`。
- 增大安全切向修正幅值，检验 `tangent_acceptance`。
- 适度降低 `dual_alpha_hard`，让高刚度区更偏力控。

## 8. 与论文第五章的对应关系

- `alpha_hr` 对应第五章参考层人机仲裁。
- `project_reference()` 对应安全投影。
- `compute_alpha_fp()` 对应力安全裕度和刚度响应的力位仲裁。
- `metrics_for()` 对应实验指标统计。
- `plot_results()` 对应论文插图生成流程。

因此，修改实验环境、输入脚本或指标权重后，应同步检查第五章中方法描述和实验结果解释是否一致。
