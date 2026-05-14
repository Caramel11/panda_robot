# 合作博弈力-位控制项目 — 使用指南

## 快速运行

```bash
# Terminal 1: 启动仿真
roslaunch panda_simulator simulation.launch

# Terminal 2: 默认配置 (coop_fuzzy 策略 + 1 次试验 + 虚拟环境)
python run_with_rcm.py

# 数据分析
python plot_results.py results/rcm_<时间戳>/
```

主程序运行时实时打印工具位置、期望位置、位置误差、RCM 误差、期望/实际/误差力、α 调度参数。每次试验结束后保存 `.npz` 数据文件，可用 `plot_results.py` 生成 6 张图。

## 控制空间架构

本版本将 RCM 模式的控制空间从 **tool-space + lever-transform** 改为 **flange-space (forward RCM mapping)**：

```
旧: tool 误差 → ARE → u_tool → 杠杆缩放 → u_flange → J_flange^T → τ
新: tool 期望 → 正映射 → flange 期望 → flange 误差 → ARE → u_flange → J_flange^T → τ
```

### 核心数学

**位置正映射**（从 tool 期望反求 flange 期望，两者通过 trocar 杠杆刚性约束）：

$$\boldsymbol{x}_{flange}^{ref} = \boldsymbol{x}_{tool}^{ref} + L \cdot \hat{\boldsymbol{n}}, \quad \hat{\boldsymbol{n}} = \frac{\boldsymbol{p}_{trocar} - \boldsymbol{x}_{tool}^{ref}}{\|\boldsymbol{p}_{trocar} - \boldsymbol{x}_{tool}^{ref}\|}$$

**速度正映射**（对位置求导，利用 $\hat{\boldsymbol{n}} \cdot \hat{\boldsymbol{n}} = 1$ 约束）：

$$\dot{\boldsymbol{x}}_{flange}^{ref} = \left(I - \frac{L}{r}(I - \hat{\boldsymbol{n}}\hat{\boldsymbol{n}}^T)\right)\dot{\boldsymbol{x}}_{tool}^{ref}, \quad r = \|\boldsymbol{p}_{trocar} - \boldsymbol{x}_{tool}^{ref}\|$$

矩阵 $(I - \hat{\boldsymbol{n}}\hat{\boldsymbol{n}}^T)$ 是工具轴垂直平面的投影算子。物理含义：

- **轴向分量**（$\hat{\boldsymbol{n}}$ 方向）：flange 与 tool 同方向同速运动
- **横向分量**（与 $\hat{\boldsymbol{n}}$ 垂直）：flange 反方向运动，按 $(L/r - 1)$ 倍缩放

在典型扫描场景下 $L/r \approx 2.3$，所以 tool 横移 5 mm/s 时 flange 反向运动 6.5 mm/s（杠杆放大效应）。

**控制律**（直接产生 flange 力，无杠杆变换）：

$$\boldsymbol{e}_{r1}^{flange} = \boldsymbol{x}_{flange} - \boldsymbol{x}_{flange}^{ref}$$
$$\boldsymbol{e}_{r2}^{flange} = \dot{\boldsymbol{x}}_{flange} - \dot{\boldsymbol{x}}_{flange}^{ref}$$
$$\boldsymbol{u}_{flange} = -\big(K_{eff}[0]\boldsymbol{e}_{r1}^{flange} + K_{eff}[1]\boldsymbol{e}_{r2}^{flange} + K_{eff}[2]\boldsymbol{e}_f + K_{eff}[3]\boldsymbol{\sigma}_f\big)$$

注意：力误差 $\boldsymbol{e}_f$ 和力积分 $\boldsymbol{\sigma}_f$ 仍以 tool 空间值进入（接触发生处的物理量），不做杠杆变换——直接累加到 flange 力上。这是新方案的核心简化。

### 增益表保持不变的依据

ARE 解依赖系统矩阵 $\bar{A}$ 中的 $-K_v/M, -c/M$ 等参数，它们刻画的是**闭环期望的二阶动力学行为**（惯性、阻尼、刚度规格），与控制点是 tool 还是 flange 无关。所以 `gt_controller.py` 中的预计算增益表对两种方案完全相同，不需要重算。

## 与初版的差异汇总

| 编号 | 项目 | 初版 | 当前版本 |
|:---:|------|------|------|
| C1 | 阻抗方程 | $M\ddot{x}+C\dot{x}=u+f$ | $M\ddot{x}+C\dot{x}+K_v(x-x_r)=u+f$ |
| C2 | $A$ 矩阵 | $A_{[2,1]}=0$ (奇异) | $A_{[2,1]}=-K_v/M$ (满秩) |
| C3 | 积分器 | 纯积分 | 泄漏积分（$\epsilon_r, \epsilon_f$） |
| C4 | ARE 维度 | 2D core + 手工积分增益 | 4D 每轴 |
| C5 | $Q$ 参数 | $q_{r2}=1, q_f=500$ 等 | 归一化：$q_{r1}=40000, q_{r2}=400, q_f=1, q_{sf}=0.25$ |
| C6 | 控制律 | 混合: $-K_{core}z_{core}+K_Ie_{r1}+K_I\sigma_f$ | 统一: $-K_{eff}z$ |
| C7 | RCM 控制空间 | tool 空间 + 杠杆缩放 | flange 空间 + 正映射 [新] |

## 实现细节

### 决策 1：`e_r1` 直接计算而非泄漏积分

将 `-eps_r·e_r1` 项仅作为 ARE 设计中的正则化项（保证 $\bar{A}$ 满秩），实际控制时取 `e_r1 = x − x_r`。增益变化 < 1%（对比 $\epsilon_r=0$ 时的 ARE 解）。

### 决策 2：力方向约定

```python
F_des  = [0, 0, +F_d]       # 接触反力 +z 向上
F_meas = [0, 0, +|F_z|]
e_f    = F_meas - F_des
```

与 $K_{ef} < 0$ 自洽：$e_f < 0$（力过小）→ $-K_{ef}e_f < 0$（z 方向控制力向下，flange 下推 → tool 通过杠杆上推增加接触）。

### 决策 3：力误差不做杠杆变换

新方案的 $\boldsymbol{e}_f$ 和 $\boldsymbol{\sigma}_f$ 直接以 tool 空间值进入控制律，产生 flange 力贡献。物理含义：把力控视为"为达到目标接触力，flange 应叠加的等效力"。这与位置控制点切换到 flange 的整体逻辑一致。如果对力相关项做杠杆变换，就回到了"tool 力换算到 flange 力"的旧逻辑，违背重构初衷。

## 项目文件

```
cooperative_gt/
├── run_with_rcm.py         主入口 A: RCM 约束模式 (flange-space)
├── run_no_rcm.py           主入口 B: 无 RCM 模式 (tool-space)
├── plot_results.py         数据分析与可视化 (6 张图)
├── fuzzy_logic.py          (你的原文件)
├── kalman_filter.py        (你的原文件)
├── GUIDE.md                本文档
└── src/
    ├── gt_controller.py    核心: 4D ARE + K_v 合作博弈控制器
    ├── alpha_scheduler_gt.py  α 调度: 阶段感知 + 模糊 + KF
    ├── env_estimator.py    环境刚度 RLS 估计
    ├── leaky_integrator.py 泄漏积分器（用于 σ_f）
    ├── robot_interface.py  机器人状态/RCM 正映射/姿态/力矩装配
    └── utils.py            虚拟环境 + 数据记录
```

## plot_results.py 输出

```bash
python plot_results.py results/rcm_20260428_103045/coop_fuzzy_t00.npz
# 或处理目录下所有试验:
python plot_results.py results/rcm_20260428_103045/
```

每个 `.npz` 文件生成 6 张 PNG（保存到 `figures/` 子目录）：

| 图 | 内容 |
|----|------|
| 1_position | 工具位置 3 轴跟踪 + 误差范数（含后 25% 平均误差标注） |
| 2_force | 接触力期望 vs 实际 + 力误差（含 RMS / 最大值标注） |
| 3_rcm | RCM 约束违反度 + 临床安全上限 1mm 参考线 |
| 4_alpha_K | α 时间序列 + K̂_e 对数尺度（带阶段切换辅助线） |
| 5_gains | 4 个 Nash 增益 (K_total, K_r2, K_ef, K_sf) 时变曲线 |
| 6_summary | 单图综合面板（6 个关键量并排展示） |

终端同时输出文本统计摘要：位置误差均值/最大值/后 25% 稳态均值、力 RMS、RCM 误差均值/峰值、α 范围。

## 快速启动

```bash
# 0. 放置项目
cp -r cooperative_gt/ ~/catkin_ws/
cd ~/catkin_ws/cooperative_gt

# 1. 启动仿真 (Terminal 1)
roslaunch panda_simulator simulation.launch

# 2. 运行 RCM 模式实验 (Terminal 2)
python run_with_rcm.py --strategy all --trials 3 --use-virtual-env

# 3. 运行无 RCM 模式实验
python run_no_rcm.py --strategy all --trials 3 --use-virtual-env

# 4. 仅运行单策略快速验证
python run_with_rcm.py --strategy coop_fuzzy --trials 1 --use-virtual-env
```

## 可选策略

- `fixed_02`：固定 α=0.2（偏力控）
- `fixed_05`：固定 α=0.5（均衡）
- `fixed_08`：固定 α=0.8（偏位控）
- `coop_fuzzy`：阶段感知 + 模糊 + KF 动态调度（推荐）
- `all`：依次运行所有策略

## 参数调整（`src/gt_controller.py`）

### 基线虚拟刚度 K_v

```python
self.Kv = 100.0    # N/m
```

- 增大：整体位置跟踪更硬，但在硬环境中可能放大力超调
- 减小：柔顺性增加，但可能出现位置漂移
- 推荐范围：50-200 N/m

### 归一化 Q 参数

```python
self.q_r1 = 40000.0  # 1 / (典型位置误差)²
self.q_r2 = 400.0    # 1 / (典型速度误差)²
self.q_f  = 1.0      # 1 / (典型力误差)²
self.q_sf = 0.25     # 1 / (典型力积分)²
```

- 典型位置误差 5mm → q_r1 = 40000
- 典型速度误差 50mm/s → q_r2 = 400
- 典型力误差 1N → q_f = 1
- 典型力积分 2N·s → q_sf = 0.25

这种归一化确保各项 $q_i \cdot e_i^2$ 在典型工况下量级相同，避免某一分量主导 LQR 的优化目标。

### 泄漏积分器

```python
self.eps_r = 1.0   # s⁻¹  位置泄漏 (时间常数 1.0s)
self.eps_f = 2.0   # s⁻¹  力积分泄漏 (时间常数 0.5s)
```

- 增大 eps：更快忘记历史误差，减少稳态偏差恢复时间，但降低积分作用
- 减小 eps：更接近纯积分器，但接近 0 时 A 矩阵趋于奇异

## 可视化分析

记录的数据包含以下字段：

```
t            时间戳
pos_{x,y,z}  工具尖端位置
F_measured   测量接触力
F_desired    目标接触力
e_f          力误差
e_r          位置误差
sigma_f_norm 力积分状态范数
e_r1_norm    位置积分状态范数 (泄漏)
alpha        仲裁参数
K_hat        RLS 刚度估计
K_total      有效位置刚度 K_v + K_r1 [新]
K_r2         速度反馈增益
K_ef         力反馈增益
K_sf         力积分反馈增益
phase        任务阶段 (0=Free, 1=Approach, 2=Trans, 3=Steady, 4=Retreat)
```

关键观察指标：

- **K_total 的自适应**：软面应 > 100（加刚保位），硬面应 < 100（减刚防超调）
- **K_ef 随 α 的单调性**：α 越小 |K_ef| 越大（力控越激进）
- **α 的阶段响应**：过渡瞬间 α 应快速跳变

## 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| ARE 报错 Failed to find finite solution | eps_r 或 eps_f 太小 / Q 权重不当 | 使用文档默认参数 |
| K_total 非单调 | alpha_grid 太稀 | 用 21 点或更密 |
| 力超调持续不减 | sigma_f 饱和 | 检查 eps_f，是否需要增大 |
| 位置漂移 | K_total 太小 | 增大 K_v 或 q_r1 |
| K_ef 接近 0 | α 几乎等于 1 | 查看 α 调度器输出 |
| 预计算耗时长 | 网格太密 | 减少 K_e_grid 点数 |
