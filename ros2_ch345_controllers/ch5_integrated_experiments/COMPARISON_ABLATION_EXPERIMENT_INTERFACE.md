# 第3/4/5章对比实验与消融实验接口说明

## 1. 实验分类

本文实验分为两类。

**对比实验**用于回答“本文方法相比常用方法是否更好”。对比对象是标准阻抗控制、传统混合力位控制、标准 MPC、固定融合/固定仲裁等常用基线。

**消融实验**用于回答“本文方法内部模块是否必要”。消融对象是固定 `alpha_FP`、去掉安全投影、只保留参考层、只保留执行层、去掉 RCM 安全投影等本文方法变体。

## 2. 统一入口

新增统一入口：

```bash
ros2 run ch5_integrated_experiments run_ch345_experiment_matrix
```

默认只生成实验计划，不启动 Gazebo。加入 `--execute` 才会实际运行。

### 2.1 生成短快 S 曲线筛查计划

```bash
source /opt/ros/humble/setup.bash
source /home/liu/franka_ros2_ws/install/setup.bash
ros2 run ch5_integrated_experiments run_ch345_experiment_matrix \
  --profile quick \
  --realistic-env
```

该命令生成：

- `ch345_experiment_matrix_plan.json`
- `CH345_COMPARISON_ABLATION_EXPERIMENT_PLAN.md`

默认短快参数：

- 场景：`ustc_smooth_s`
- 重复次数：1
- 任务时长：18 s
- no-RCM 轨迹尺度：0.35
- with-RCM 轨迹尺度：0.20
- 真实环境模型：`sponge_300_600_two_block`
- 人类输入：`realistic_events`

### 2.2 执行某一项实验

例如只运行第五章 RCM 对比实验：

```bash
source /opt/ros/humble/setup.bash
source /home/liu/franka_ros2_ws/install/setup.bash
ros2 run ch5_integrated_experiments run_ch345_experiment_matrix \
  --profile quick \
  --case-ids ch5_comparison_with_rcm \
  --realistic-env \
  --execute
```

### 2.3 生成论文完整 U/S/T/C 三重复计划

```bash
source /opt/ros/humble/setup.bash
source /home/liu/franka_ros2_ws/install/setup.bash
ros2 run ch5_integrated_experiments run_ch345_experiment_matrix \
  --profile paper \
  --trials 3 \
  --realistic-env
```

`paper` 模式默认运行 `ustc_smooth_u/s/t/c` 四种字母轨迹。

## 3. 当前完整实验矩阵

### 3.1 第三章对比实验

接口：

```bash
ros2 run ch3_experiments run_ch3_smooth_letter_comparison
```

方法：

- `standard_impedance`：标准阻抗控制；
- `traditional_hybrid`：传统混合力位控制；
- `standard_mpc`：标准 MPC；
- `balanced_fixed`：固定仲裁；
- `continuous_force_margin`：本文动态力位协调方法。

目的：证明本文执行层方法在刚度切换柔性接触中，相比常用执行层控制器能更好兼顾位置误差和接触力安全。

### 3.2 第三章消融实验

接口：

```bash
ros2 run ch3_experiments run_ch3_smooth_letter_comparison
```

消融一：固定 `alpha_FP`

- `strong_position`
- `balanced_fixed`
- `strong_force`
- `continuous_force_margin`

消融二：`continuous_force_margin` 参数组

- `stable`
- `stiffness_sensitive`
- `sensitive`
- `alpha_stiffness`

目的：验证动态 `alpha_FP`、刚度敏感项和力安全裕度调节对降低力位综合误差的必要性。

### 3.3 第四章对比实验

接口：

```bash
ros2 run ch4_shared_gt_controller run_ch4_smooth_letter_comparison
```

方法：

- `direct_accept`：直接接受人类输入；
- `fixed_blend`：固定融合；
- `single_sigmoid`：单通道 Sigmoid 仲裁；
- `full_method`：本文安全动态仲裁。

目的：证明本文参考层方法能够保留安全切向修正，并抑制危险法向输入。

### 3.4 第四章消融实验

接口：

```bash
ros2 run ch4_shared_gt_controller run_ch4_smooth_letter_comparison
```

方法：

- `direct_accept`
- `fixed_blend`
- `single_sigmoid`
- `dynamic_no_projection`
- `full_method`

目的：验证安全投影、方向分解和动态人类权重的必要性。

### 3.5 第五章 no-RCM 综合对比实验

接口：

```bash
ros2 run ch5_integrated_experiments run_ch5_smooth_letter_comparison
```

方法：

- `standard_impedance`
- `traditional_hybrid`
- `standard_mpc`
- `balanced_fixed`
- `full_method`

目的：验证工业柔性表面扫描任务中，完整双层方法相比常用控制器的综合优势。

### 3.6 第五章 no-RCM 综合消融实验

方法：

- `balanced_fixed`
- `reference_only`
- `execution_only`
- `no_projection`
- `full_method`

目的：验证参考层、执行层和安全投影缺一不可。

### 3.7 第五章 with-RCM 综合对比实验

方法：

- `rcm_standard_impedance`
- `rcm_traditional_hybrid`
- `rcm_standard_mpc`
- `rcm_balanced_fixed`
- `rcm_full_method`

目的：验证长工具固定孔约束下，本文完整方法在轨迹误差、接触力安全和 RCM 误差之间的综合优势。

### 3.8 第五章 with-RCM 综合消融实验

方法：

- `rcm_balanced_fixed`
- `rcm_reference_only`
- `rcm_execution_only`
- `rcm_no_projection`
- `rcm_full_method`

目的：验证 RCM 约束下参考层、执行层、安全投影和 RCM 稳定处理的必要性。

## 4. 推荐实验计划

### 阶段 A：短快可行性筛查

只运行 `S` 曲线，每组 1 次：

```bash
ros2 run ch5_integrated_experiments run_ch345_experiment_matrix \
  --profile quick \
  --scenarios ustc_smooth_s \
  --realistic-env \
  --execute
```

若全部控制器稳定，再进入阶段 B。

### 阶段 B：论文正式对比实验

运行 U/S/T/C，每组 3 次：

```bash
ros2 run ch5_integrated_experiments run_ch345_experiment_matrix \
  --profile paper \
  --trials 3 \
  --realistic-env \
  --execute
```

### 阶段 C：单项补跑

若某一项指标不理想，只补跑对应 case，例如：

```bash
ros2 run ch5_integrated_experiments run_ch345_experiment_matrix \
  --profile quick \
  --case-ids ch3_comparison_no_rcm \
  --realistic-env \
  --execute
```

## 5. 数据记录要求

每个实验至少保留以下数据：

- `result.npz`
- `summary.json`
- 方法级指标 CSV
- 综合报告 Markdown
- 轨迹图
- 接触力时序图
- 轨迹误差时序图
- RCM 误差时序图，仅 with-RCM
- 人类输入与安全投影图，仅第4/5章
- `alpha_FP`、`alpha_HR`、`rcm_reference_scale` 时序图

## 6. 论文写作对应关系

- 第三章对比实验：证明执行层动态力位协调优于标准执行层控制器；
- 第三章消融实验：证明动态 `alpha_FP` 和刚度/力裕度调节必要；
- 第四章对比实验：证明参考层动态仲裁优于直接接受、固定融合和单 Sigmoid；
- 第四章消融实验：证明安全投影和方向分解必要；
- 第五章对比实验：证明双层方法在工业和 RCM 场景中有综合优势；
- 第五章消融实验：证明参考层、执行层和 RCM 稳定处理共同构成完整贡献。
