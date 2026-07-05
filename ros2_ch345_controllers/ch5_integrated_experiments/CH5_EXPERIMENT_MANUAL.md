# 第五章综合实验撰写与代码使用说明

## 1. 第五章应如何撰写

第 5 章不建议再提出新的控制理论，而应写成系统综合验证章。章节逻辑是：第 3 章解决执行层“力跟踪与位姿跟踪如何协调”，第 4 章解决参考层“人类输入如何被安全接受、抑制和融合”，第 5 章验证两层同时启用后是否仍能稳定运行、误差收敛，并证明综合方法优于单层方法和固定权重方法。

建议标题结构：

1. 综合实验目的与系统结构。
2. 第 3 章执行层与第 4 章参考层的统一控制链。
3. 实验平台、场景、数据采集和评价指标。
4. Gazebo 在线综合实验。
5. 消融实验：固定融合、仅参考层、仅执行层、完整方法。
6. 实物接口与安全预检。
7. 结果分析与本章小结。

正文中应避免只罗列程序运行结果。每一组图表都要回答一个论文问题：

- 动态参考层是否保留切向有益输入并抑制法向危险输入。
- 动态执行层是否根据力裕度改变 `alpha_FP`，在力风险上升时增加力约束优先级。
- 两层同时启用后，末段轨迹误差是否收敛到允许范围内。
- 消融方法失败或性能下降的原因是什么。

## 2. 需要进行的实验

### 2.1 Gazebo 在线综合实验

必做方法：

- `standard_impedance`：第 4 章完整参考层 + 标准阻抗执行层。
- `traditional_hybrid`：第 4 章完整参考层 + 传统混合力位执行层。
- `standard_mpc`：第 4 章完整参考层 + 标准 MPC/QP 风格执行层。
- `balanced_fixed`：第 4 章固定参考融合，第 3 章固定力/位置优先级。
- `reference_only`：只启用第 4 章动态参考仲裁，执行层保持固定。
- `execution_only`：只启用第 3 章动态执行优先级，参考层不接受人类修正。
- `full_method`：同时启用第 4 章动态参考仲裁和第 3 章动态执行优先级。

必做场景：

- `mixed_sequence`：先有切向有益输入，再有法向风险输入，最后回到可跟踪参考。
- `no_rcm`：用于验证普通柔性操作任务。
- `with_rcm`：用于验证受远心约束任务。当前包已保留指标接口，实测前需要确认上游 Gazebo/控制器的 `with_rcm` 启动配置可用。

推荐重复次数：

- 论文正式数据至少每个方法 3 次。
- 调参阶段可以先每个方法 1 次。

### 2.2 消融实验

第 5 章至少需要以下对照：

| 对照 | 要证明的问题 |
|---|---|
| `standard_impedance` vs `full_method` | 普通阻抗/位置优先执行层缺少法向力主动调节，复杂接触中力峰值或越界时间可能更差 |
| `traditional_hybrid` vs `full_method` | 传统混合力位能直接闭合法向力，但固定方向解耦缺少阶段自适应与参考层协同 |
| `standard_mpc` vs `full_method` | 预测约束能在越界附近保守修正，但缺少连续可解释的 `alpha_FP` 仲裁趋势 |
| `balanced_fixed` vs `full_method` | 固定权重无法同时兼顾人类输入接受和力安全约束 |
| `reference_only` vs `full_method` | 只有参考层时执行层不会根据力裕度调整 `alpha_FP` |
| `execution_only` vs `full_method` | 只有执行层时不能体现人类切向修正的参考收益 |
| `no_projection` vs `full_method` | 没有安全投影时法向危险输入更容易引入力风险 |

### 2.3 实物接口实验

实物实验建议分三步：

1. Falcon 独立测试：确认 `/falcon/ee_pose`、`/falcon/velocity`、`/falcon/joystick`、`/falcon/applied_force` 正常。
2. 机器人接口预检：确认 `/joint_states` 有新消息，`/no_rcm_effort_controller/commands` 有 controller subscriber。
3. 第 5 章综合控制：先 `enable_control:=false` 或低增益观察日志，再允许 effort 命令进入机器人。

本包新增 `ch5_hardware_preflight`，用于第 1、2 步自动检查。

本次进一步补充了 Falcon 到第 5 章控制链的桥接接口：

- `ch5_falcon_human_bridge`：订阅 `/falcon/ee_pose`、`/falcon/velocity`、`/falcon/joystick`，发布 `/ch5/human_delta`。
- 第 4 章控制器新增 `scenario:=external`，从 `/ch5/human_delta` 接收外部人类输入。
- 输入有超时失效和幅值限幅，Falcon 停止发布或松开 clutch 时不会持续沿用旧输入。

## 3. 需要准备的数据

每次实验至少保存以下字段。当前 `ch4_shared_gt_controller` 已写入这些字段，第 5 章包直接读取 `result.npz`：

- 时间：`t`。
- 参考轨迹：`nominal_ref_*`、`tool_ref_*`。
- 人类候选参考与安全参考：`x_h_*`、`x_h_safe_*`。
- 实际末端轨迹：`tool_*`。
- 误差：`tracking_error`、`rcm_error`。
- 第 4 章变量：`alpha`、`rho_F`、`kappa_N`、`beta`、`I_h`、`T_h`、`C_h`。
- 第 3 章变量：`alpha_FP`、`alpha_FP_target`。
- 力安全变量：`y_f`、`F_min`、`F_d`、`F_max`。

第 5 章统一指标：

- 收敛误差：`tracking_last_rms_m`、`tracking_last_max_m`。
- RCM 误差：`rcm_last_rms_m`、`rcm_last_max_m`。
- 人类切向接受率：`R_acc`。
- 法向危险抑制率：`R_sup`。
- 力越界时间：`T_vio_s`。
- 力峰值偏差：`F_peak_N`。
- 仲裁权重均值和平滑性：`alpha_HR_mean`、`S_alpha_HR`、`alpha_FP_mean`、`S_alpha_FP`。
- 综合分数：`score`，越小越好。

## 4. 如何利用已有代码完成

第 5 章包复用现有第 4 章 Gazebo launch：

```bash
ros2 launch ch4_shared_gt_controller ch4_shared_gt_gazebo.launch.py \
  task_mode:=no_rcm \
  controller_variant:=gt_kf \
  arbitration_strategy:=full_method \
  execution_strategy:=dynamic_gt \
  scenario:=mixed_sequence
```

第 5 章自动编排脚本会替你循环调用该 launch，并根据方法名自动切换：

- `arbitration_strategy`：第 4 章参考层策略。
- `execution_strategy`：第 3 章执行层策略。
- `fixed_alpha_hr`：固定参考层权重。
- `fixed_alpha_fp`：固定执行层权重。

推荐正式命令：

```bash
cd /home/liu/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run ch5_integrated_experiments run_ch5_gazebo_suite \
  --task-mode no_rcm \
  --scenario mixed_sequence \
  --methods standard_impedance,traditional_hybrid,standard_mpc,balanced_fixed,reference_only,execution_only,no_projection,full_method \
  --trials 3 \
  --timeout-s 95
```

聚合多组结果并导出论文表格：

```bash
ros2 run ch5_integrated_experiments ch5_aggregate_results \
  --inputs /home/liu/franka_ros2_ws/results/ch5_integrated \
  --output-dir /home/liu/franka_ros2_ws/results/ch5_integrated/aggregate \
  --expected-methods balanced_fixed,reference_only,execution_only,no_projection,full_method
```

实物/Falcon 接入命令：

```bash
ros2 launch ros2_falcon falcon.launch.py
ros2 launch ch5_integrated_experiments ch5_falcon_bridge.launch.py
ros2 launch ch4_shared_gt_controller ch4_shared_gt_gazebo.launch.py \
  scenario:=external \
  arbitration_strategy:=full_method \
  execution_strategy:=dynamic_gt \
  external_human_topic:=/ch5/human_delta
```

## 5. 本次新增和修改的代码

### 5.1 修改 `ch4_shared_gt_controller`

`shared_gt_gazebo_node.py` 新增执行层策略参数：

- `execution_strategy`：`fixed_02`、`fixed_05`、`fixed_08`、`dynamic_gt` 等。
- `fixed_alpha_fp`：固定执行层权重。
- `alpha_fp_tau_s`：动态 `alpha_FP` 一阶滤波时间常数。

控制律层面新增：

- 根据力裕度 `rho_F` 和力误差生成 `alpha_FP_target`。
- 用一阶滤波得到 `alpha_FP`。
- 在法向 PD 增益中引入 `alpha_FP`，使位置优先级和力优先级可连续变化。
- 在 `dynamic_gt` 中加入小幅法向力释放项，帮助力风险上升时减小法向压力。
- 新增 `standard_impedance`：三轴任务空间弹簧阻尼，不显式闭合法向力环，用作标准阻抗控制对照。
- 新增 `traditional_hybrid`：切向位置控制、法向力误差反馈，用作传统混合力位控制对照。
- 新增 `standard_mpc`：短预测窗 + 力边界投影修正，用作标准 MPC/QP 风格对照。
- 日志增加 `alpha_FP`、`alpha_FP_target`、`execution_strategy`。
- 新增 `external_human_topic`、`external_human_timeout_s`、`external_human_scale`、`external_human_max_delta_m` 参数。
- 新增 `scenario:=external/falcon_external/live_human`，用于从外部 ROS2 topic 接收人类参考增量。

`ch4_shared_gt_gazebo.launch.py` 新增：

- `execution_strategy` launch 参数。
- `fixed_alpha_fp` launch 参数。

结果目录名已包含 `arbitration_strategy` 和 `execution_strategy`，例如：

```text
no_rcm_gt_kf_full_method_dynamic_gt_mixed_sequence_20260702_040840
```

### 5.2 新增 `ch5_integrated_experiments`

新增模块：

- `method_switches.py`：定义第 5 章方法矩阵。
- `run_ch5_gazebo_suite.py`：启动 Gazebo、监视日志、收集单次结果、生成报告。
- `ch5_metrics.py`：从 `result.npz` 和 `summary.json` 计算统一指标。
- `ch5_plotting.py`：生成柱状图和典型时序图。
- `generate_ch5_commands.py`：生成常用命令。
- `hardware_preflight.py`：实物实验前检查 Falcon 与机器人接口话题。
- `falcon_human_bridge.py`：把 Falcon 主端位移转换为第 4 章候选参考增量。
- `aggregate_results.py`：聚合多次实验，输出均值/标准差、LaTeX 表格和查漏补缺报告。

## 6. 已完成的在线 Gazebo 验证

### 6.1 四方法综合验证

结果目录：

```text
/home/liu/franka_ros2_ws/results/ch5_integrated/ch5_gazebo_no_rcm_mixed_sequence_20260702_040359
```

关键结果：

| method | success | tracking RMS | tracking max | force peak | R_acc | R_sup | alpha_FP_mean |
|---|---:|---:|---:|---:|---:|---:|---:|
| `balanced_fixed` | True | 2.69 mm | 2.97 mm | 0.171 N | 0.500 | 0.500 | 0.500 |
| `reference_only` | True | 2.81 mm | 3.59 mm | 0.068 N | 0.572 | 0.793 | 0.500 |
| `execution_only` | False | 3.09 mm | 3.12 mm | 0.000 N | 0.000 | 1.000 | 0.832 |
| `full_method` | True | 2.80 mm | 3.56 mm | 0.068 N | 0.572 | 0.793 | 0.831 |

解释：

- `full_method` 满足 3 mm RMS 和 4.5 mm 最大误差判据，力越界时间为 0。
- `reference_only` 证明第 4 章参考层可提升人类输入接受与危险输入抑制，但执行层 `alpha_FP` 不动态变化。
- `execution_only` 超过严格 3 mm RMS 判据，符合消融预期：只靠执行层无法利用人类切向修正。
- `full_method` 同时具有动态 `alpha_HR` 和动态 `alpha_FP`，是第 5 章应重点展示的方法。

### 6.3 安全投影消融验证

结果目录：

```text
/home/liu/franka_ros2_ws/results/ch5_integrated/ch5_gazebo_no_rcm_mixed_sequence_20260702_041758
```

`no_projection` 指标：

- `success=False`
- `tracking_last_rms_m=0.309388`，即 309.39 mm。
- `tracking_last_max_m=0.309388`，即 309.39 mm。
- `F_peak_N=0.1700`
- `R_acc=0.5719`
- `R_sup=0.6163`
- `alpha_FP_mean=0.8303`

解释：该方法关闭安全投影后，参考层仍能接受切向输入，但法向危险输入抑制率从完整方法的约 0.793 降到约 0.616，并导致末段误差发散到 0.3 m 量级。因此论文中可以把它作为“安全投影不可删除”的强消融证据。

### 6.4 当前聚合报告

聚合输出：

```text
/home/liu/franka_ros2_ws/results/ch5_integrated/aggregate/ch5_aggregate_metrics.csv
/home/liu/franka_ros2_ws/results/ch5_integrated/aggregate/ch5_paper_table.tex
/home/liu/franka_ros2_ws/results/ch5_integrated/aggregate/CH5_AGGREGATE_REPORT.md
```

当前已验证方法：

- `balanced_fixed`：可运行，满足收敛，用作固定权重基线。
- `reference_only`：可运行，满足收敛，用作第 4 章参考层贡献对照。
- `execution_only`：可运行，但严格 RMS 判据不通过，用作只启用执行层的消融劣化。
- `no_projection`：可运行，但明显不收敛，用作安全投影必要性的消融劣化。
- `full_method`：可运行且两次验证均满足收敛，是第 5 章主方法。

### 6.2 完整方法复验

结果目录：

```text
/home/liu/franka_ros2_ws/results/ch5_integrated/ch5_gazebo_no_rcm_mixed_sequence_20260702_040805
```

复验指标：

- `success=True`
- `tracking_last_rms_m=0.002803`，即 2.80 mm。
- `tracking_last_max_m=0.003569`，即 3.57 mm。
- `T_vio_s=0.0`
- `F_peak_N=0.0679`
- `R_acc=0.5718`
- `R_sup=0.7927`
- `alpha_HR_mean=0.3090`
- `alpha_FP_mean=0.8314`

对应图表：

```text
/home/liu/franka_ros2_ws/results/ch5_integrated/ch5_gazebo_no_rcm_mixed_sequence_20260702_040805/figures/ch5_metrics_bars.png
/home/liu/franka_ros2_ws/results/ch5_integrated/ch5_gazebo_no_rcm_mixed_sequence_20260702_040805/figures/ch5_representative_timeseries.png
```

## 7. 还需要补充的代码

当前已完成第 5 章 `no_rcm` 在线综合实验闭环。若论文需要更完整的实物和 RCM 结果，建议继续补充：

1. `with_rcm` Gazebo launch 的稳定入口，保证 RCM Jacobian、RCM 点和 command topic 与第 5 章脚本一致。
2. 实物安全状态机的更完整版本，包含急停、机器人低增益预热、自动归零、控制器切换和硬件限位联锁。
3. 正式论文数据需要每个方法至少 3 次重复；当前已有完整方法 2 次，其余方法 1 次。
4. 如果要做统计显著性检验，还需为 `ch5_aggregate_results` 补充 t 检验或非参数检验。
5. 论文图表可继续按学校模板调整字体、字号和中英文标题。

## 8. 论文中可直接使用的结论表述

在 `mixed_sequence` 场景下，完整方法将第 4 章动态参考仲裁与第 3 章动态执行优先级同时启用。Gazebo 在线结果显示，系统末段轨迹误差 RMS 为 2.80 mm，最大误差为 3.57 mm，均满足 3 mm 收敛阈值及 4.5 mm 最大误差约束；力代理量未发生越界，力峰值偏差为 0.068 N。与此同时，人类切向输入接受率约为 0.572，法向危险输入抑制率约为 0.793，说明系统能够保留有益的人类修正并抑制可能造成力风险的法向输入。执行层 `alpha_FP` 均值约为 0.831，表明在该场景中控制器主动提高位置/力协调中的安全执行权重；参考层 `alpha_HR` 在任务过程中连续变化，说明共享控制权重不是固定融合，而是随意图与风险在线调整。
