# 第四章实验代码与方法手册

## 1. 目标与边界

本包用于第4章“面向人机协同柔性操作的动态仲裁方法”的 ROS2/Gazebo 验证。实验边界为参考层仲裁：第三章执行层控制器、Gazebo 机器人接口、Pinocchio 运动学和 effort command topic 保持固定，只改变人类输入如何生成、筛选、仲裁并进入最终期望轨迹。

本包由 `shared_gt_controller` 克隆得到，原包未继续修改。克隆包路径：

```text
/home/liu/franka_ros2_ws/src/ch4_shared_gt_controller
```

控制器源码的模块划分、主要函数和论文变量对应关系见：

```text
CH4_CONTROLLER_CODE_GUIDE.md
```

## 2. 数学方法与代码对应

### 2.1 自主参考 `x_r`

论文含义：机器人自主扫描参考，作为无人输入或人类输入被抑制时的基准轨迹。

代码位置：

```text
ch4_shared_gt_controller/shared_gt_gazebo_node.py::_nominal_reference()
```

实现说明：使用 Gazebo 可重复局部四段轨迹。no-RCM 中加入 x 方向扫描分量；with-RCM 保留较小局部轨迹，便于边界验证。

日志字段：

```text
nominal_ref_0, nominal_ref_1, nominal_ref_2
```

### 2.2 主端输入与直接遥操作参考 `x_tele`

论文公式：

```text
x_tele = x_r + K_m Delta x_m
```

代码位置：

```text
_human_delta()
_candidate_reference()
```

当前 Gazebo 验证不依赖真实 Falcon，而是用预设输入序列复现四类人类输入：

- `tangential_correction`：安全切向修正。
- `unsafe_normal_push`：危险法向压入。
- `short_pulse_disturbance`：短时扰动。
- `sustained_intervention`：持续明确干预。
- `mixed_sequence`：四类输入组合，用于一轮综合验证。

日志字段：

```text
x_tele_0, x_tele_1, x_tele_2
F_h, I_h
```

### 2.3 人侧候选参考 `x_h` 与 `beta`

论文公式：

```text
x_h = beta x_tele + (1-beta) x_r
beta_r = sigmoid(gamma_beta (||x_tele-x_r|| - d_th))
dot beta = (beta_r - beta) / tau_beta
```

代码位置：

```text
_candidate_reference()
```

日志字段：

```text
x_h_0, x_h_1, x_h_2
beta
```

### 2.4 安全参考投影 `x_h_safe`

论文含义：先预测候选参考是否会带来接触力风险，再对法向分量进行裁剪，尽量保留切向分量。

简化预测模型：

```text
F_hat(x_bar) = F_d + K_hat n_down^T (x_bar - x_r)
```

其中 `n_down=[0,0,-1]`，向下压入使预测接触力升高。

投影逻辑：

```text
delta = x_h - x_r
delta_N = (delta^T n_down) n_down
delta_T = delta - delta_N
delta_N_safe = clip(delta_N, force-safe interval)
x_h_safe = x_r + delta_T + delta_N_safe
```

代码位置：

```text
_predict_force()
_safe_project_reference()
```

日志字段：

```text
x_h_safe_0, x_h_safe_1, x_h_safe_2
rho_F
kappa_N
y_f_candidate
y_f_safe
normal_raw
normal_safe
```

### 2.5 动态人机仲裁 `alpha_HR`

论文含义：`alpha_HR` 决定安全人侧参考进入最终期望轨迹的比例。数值越大，人侧监督影响越强。

代码位置：

```text
arbitration.py::SharedFuzzyArbitrator
shared_gt_gazebo_node.py::_alpha_hr()
```

输入特征：

```text
I_h/F_h     输入强度
T_h         有效输入持续时间
D_r/D_c     安全距离或风险代理量
C_h         方向一致性，记录用于分析
```

日志字段：

```text
alpha
alpha_dynamic
alpha_raw
delta_lambda
T_h
C_h
D_c
```

### 2.6 最终期望轨迹 `x_d`

完整方法：

```text
x_d = x_r + alpha_HR P_T (x_h_safe-x_r)
          + alpha_HR kappa_N P_N (x_h_safe-x_r)
```

其中 `P_T` 表示切向分量，`P_N` 表示法向分量。接触力接近边界时，`kappa_N` 下降，从而削弱危险法向压入。

代码位置：

```text
_reference()
```

日志字段：

```text
tool_ref_0, tool_ref_1, tool_ref_2
```

## 3. 对比策略

参数名：

```text
arbitration_strategy
```

支持策略：

```text
autonomous_only
direct_accept
fixed_blend
single_sigmoid
dynamic_no_projection
full_method
```

策略含义：

- `autonomous_only`：`x_d=x_r`，用于无人输入基线。
- `direct_accept`：`x_d=x_tele`，用于显示未筛选输入的风险。
- `fixed_blend`：固定比例融合 `x_h` 和 `x_r`。
- `single_sigmoid`：只根据输入偏离程度生成 `alpha_HR`。
- `dynamic_no_projection`：启用动态权重，但不做安全投影。
- `full_method`：启用动态权重、安全投影、切向/法向分解和 `kappa_N`。

## 4. 论文实验与代码的一一对应

| 论文实验 | scenario | 推荐策略 | 主要观察字段 | 预期现象 |
| --- | --- | --- | --- | --- |
| 安全切向修正 | `tangential_correction` | `direct_accept`, `fixed_blend`, `single_sigmoid`, `full_method` | `R_acc`, `x_d-y`, `alpha`, `y_f` | 完整方法保持较高切向接受率，力不越界 |
| 危险法向压入 | `unsafe_normal_push` | `direct_accept`, `dynamic_no_projection`, `full_method` | `R_sup`, `T_vio_s`, `F_peak_N`, `normal_safe` | 完整方法明显提高法向抑制率并降低力峰值 |
| 短时扰动 | `short_pulse_disturbance` | `direct_accept`, `single_sigmoid`, `full_method` | `alpha_max`, `S_alpha`, `tool_ref` 跳变量 | 完整方法抑制权重突跳和参考突变 |
| 持续干预 | `sustained_intervention` | `autonomous_only`, `fixed_blend`, `full_method` | `alpha`, `beta`, `R_acc`, tracking | 完整方法平滑接受持续明确输入 |
| 综合工况 | `mixed_sequence` | 全部策略 | 所有 summary 指标 | 完整方法在安全和跟踪之间取得折中 |

## 5. 构建与运行

构建：

```bash
source /opt/ros/humble/setup.bash
cd /home/liu/franka_ros2_ws
colcon build --packages-select ch4_shared_gt_controller --symlink-install
source install/setup.bash
```

运行完整方法：

```bash
ros2 launch ch4_shared_gt_controller ch4_shared_gt_gazebo.launch.py \
  task_mode:=no_rcm controller_variant:=gt_kf \
  arbitration_strategy:=full_method scenario:=mixed_sequence
```

运行直接接受基线：

```bash
ros2 launch ch4_shared_gt_controller ch4_shared_gt_gazebo.launch.py \
  task_mode:=no_rcm controller_variant:=gt_kf \
  arbitration_strategy:=direct_accept scenario:=mixed_sequence
```

生成全套命令：

```bash
ros2 run ch4_shared_gt_controller generate_ch4_commands
```

只生成综合工况的六种策略：

```bash
ros2 run ch4_shared_gt_controller generate_ch4_commands \
  --scenarios mixed_sequence \
  --strategies autonomous_only direct_accept fixed_blend single_sigmoid dynamic_no_projection full_method
```

批量启动 Gazebo 实验矩阵：

```bash
ros2 run ch4_shared_gt_controller run_ch4_gazebo_suite \
  --scenarios mixed_sequence \
  --strategies autonomous_only direct_accept fixed_blend single_sigmoid dynamic_no_projection full_method \
  --task-mode no_rcm --controller-variant gt_kf --task-duration-s 24.0
```

先检查将要执行的 launch 命令而不启动 Gazebo：

```bash
ros2 run ch4_shared_gt_controller run_ch4_gazebo_suite \
  --scenarios mixed_sequence --strategies full_method direct_accept --dry-run
```

## 6. 对比实验代码

推荐使用 `run_ch4_comparison_experiments.py` 固化论文对比实验矩阵。该脚本会按实验计划顺序启动 Gazebo，保存每次运行的 `summary.json`，然后自动按场景和总表重算窗口指标、生成对比图、导出论文资产。

代码位置：

```text
ch4_shared_gt_controller/run_ch4_comparison_experiments.py
```

实验 profile：

| profile | 实验规模 | 用途 |
| --- | --- | --- |
| `core` | 2 个场景、8 次运行 | 快速确认完整方法、无投影动态方法、直接接受等关键基线可行性 |
| `paper` | 5 个论文实验、26 次运行 | 推荐论文正式实验矩阵，覆盖安全切向、危险法向、短扰动、持续干预、综合工况 |
| `all` | 5 个场景 × 6 个策略，30 次运行 | 全量消融，适合最终补充材料或附录 |

`paper` profile 的具体矩阵：

| 实验编号 | scenario | strategies | 主要指标 |
| --- | --- | --- | --- |
| E1 | `tangential_correction` | `autonomous_only`, `direct_accept`, `fixed_blend`, `single_sigmoid`, `full_method` | `R_acc`, `tracking_last_rms_m`, `T_vio_s`, `S_alpha` |
| E2 | `unsafe_normal_push` | `direct_accept`, `fixed_blend`, `single_sigmoid`, `dynamic_no_projection`, `full_method` | `R_sup`, `T_vio_normal_s`, `F_peak_normal_N`, `safe_projection_delta_max_m` |
| E3 | `short_pulse_disturbance` | `direct_accept`, `fixed_blend`, `single_sigmoid`, `dynamic_no_projection`, `full_method` | `reference_jump_max_m`, `S_alpha`, `alpha_peak_pulse`, `tracking_last_rms_m` |
| E4 | `sustained_intervention` | `autonomous_only`, `fixed_blend`, `single_sigmoid`, `dynamic_no_projection`, `full_method` | `R_acc`, `alpha_mean_sustained`, `tracking_last_rms_m`, `T_vio_s` |
| E5 | `mixed_sequence` | 全部 6 个策略 | `tracking_last_rms_m`, `R_acc`, `R_sup`, `T_vio_s`, `F_peak_N`, `S_alpha` |

先输出全部实验命令，不启动 Gazebo：

```bash
ros2 run ch4_shared_gt_controller run_ch4_comparison_experiments \
  --profile paper --dry-run
```

快速核心验证：

```bash
ros2 run ch4_shared_gt_controller run_ch4_comparison_experiments \
  --profile core --task-mode no_rcm --controller-variant gt_kf
```

论文推荐完整矩阵：

```bash
ros2 run ch4_shared_gt_controller run_ch4_comparison_experiments \
  --profile paper --task-mode no_rcm --controller-variant gt_kf \
  --task-duration-s 24.0 --repeats 1
```

全量消融矩阵：

```bash
ros2 run ch4_shared_gt_controller run_ch4_comparison_experiments \
  --profile all --task-mode no_rcm --controller-variant gt_kf \
  --task-duration-s 24.0 --repeats 1
```

输出目录结构：

```text
results/ch4_shared_gt_gazebo/suite_<suite_id>/
  ch4_comparison_experiment_manifest.json
  <每次 Gazebo 运行的 result.npz/summary.json/图像>
results/ch4_shared_gt_gazebo/comparison_<suite_id>/
  all/ch4_metrics_summary.csv
  all/ch4_strategy_comparison.png
  <scenario>/ch4_metrics_summary.csv
  <scenario>/ch4_strategy_comparison.png
panda_robot_gt_controller_dev/tests/0630thesis/generated/ch4_assets/
  figures/
  tables/
  metrics/
  data/
```

## 7. 数据处理与论文资产导出

单次运行结束后，控制器会写入：

```text
result.npz
summary.json
ch4_shared_gt_result.png
plot_summary.json
```

按第4章实验时间窗重算指标：

```bash
ros2 run ch4_shared_gt_controller recompute_ch4_metrics \
  --input /home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo/<run_dir>
```

对多组实验生成对比表和对比图：

```bash
ros2 run ch4_shared_gt_controller compare_ch4_results \
  --runs <run_dir_1> <run_dir_2> \
  --recompute-window-metrics \
  --output-dir /home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo/comparison_ch4
```

导出论文可引用资产：

```bash
ros2 run ch4_shared_gt_controller export_ch4_paper_assets \
  --runs <run_dir_1> <run_dir_2> \
  --comparison-dir /home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo/comparison_ch4 \
  --paper-dir /home/liu/franka_ros2_ws/panda_robot_gt_controller_dev/tests/0630thesis/generated/ch4_assets
```

新增窗口指标含义：

| 指标 | 含义 | 论文用途 |
| --- | --- | --- |
| `tracking_last_rms_m` | 18-24 s 稳态窗口末端跟踪 RMS | 验证理想 3 mm 或模拟实物 10 mm 收敛要求 |
| `R_acc` | 安全切向/持续输入接受率 | 说明人类有效意图保留程度 |
| `R_sup` | 危险法向输入在最终参考中的抑制率 | 说明安全抑制能力 |
| `R_project` | 安全投影层对危险法向输入的抑制率 | 分离投影贡献 |
| `T_vio_s` | 接触力代理量越界总时间 | 验证安全约束 |
| `T_vio_normal_s` | 危险法向窗口内越界时间 | 对应 unsafe normal push 实验 |
| `F_peak_N` | 相对期望力的最大偏差 | 描述冲击峰值 |
| `S_alpha` | `alpha_HR` 变化率 RMS | 描述权重平滑性 |
| `reference_jump_max_m` | 相邻采样参考最大跳变 | 检查短扰动抑制和平滑性 |
| `safe_projection_delta_max_m` | 投影前后人侧参考最大差值 | 衡量安全投影介入强度 |

## 8. 绘图与汇总

单次实验绘图：

```bash
ros2 run ch4_shared_gt_controller plot_shared_gt_result --input <run_dir>
```

多次实验汇总：

```bash
ros2 run ch4_shared_gt_controller compare_ch4_results \
  --root /home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo
```

输出：

```text
comparison/ch4_metrics_summary.csv
comparison/ch4_strategy_comparison.png
```

## 9. 成功判据

Gazebo 控制器稳定性：

```text
tracking_last_rms_m <= 0.003  # 理想 Gazebo；模拟实物实验为 <= 0.010
tracking_last_max_m <= 0.0045 # 理想 Gazebo；模拟实物实验为 <= 0.015
```

第四章参考层效果：

```text
T_vio_s 越小越好，完整方法应为 0 或明显低于 direct_accept
F_peak_N 越小越好，完整方法应明显低于 direct_accept
R_sup 越大越好，完整方法应高于 dynamic_no_projection/direct_accept
R_acc 在安全切向工况中应保持较高，避免过度保守
S_alpha 应避免短时扰动导致尖峰
```

## 10. 已验证结果

已完成本地 Gazebo 验证：

```text
full_method + mixed_sequence
result: /home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo/no_rcm_gt_kf_full_method_mixed_sequence_20260702_035207
tracking_last_rms = 2.81 mm
R_sup = 0.793
T_vio = 0.00 s
F_peak = 0.068 N
success = true
```

对照基线：

```text
direct_accept + mixed_sequence
result: /home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo/no_rcm_gt_kf_direct_accept_mixed_sequence_20260702_034702
tracking_last_rms = 2.58 mm
T_vio = 0.72 s
F_peak = 0.560 N
success = false
```

解释：直接接受基线在末端跟踪上可以收敛，但危险法向输入未被投影，导致接触力代理量超过软安全上界。完整方法牺牲少量切向接受率，换取零力越界和稳定收敛，符合第4章实验目标。
