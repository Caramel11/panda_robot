# Force-Margin Alpha Arbitration Design

本文档说明 `cooperative_gt_0428` 当前默认使用的 `force_margin` alpha 仲裁方案。该方案实现于 `src/alpha_scheduler_gt.py` 的 `ForceMarginFuzzyAlphaScheduler`，并已接入 `run_with_rcm.py` 与 `run_no_rcm.py`。

## 1. 设计目标

合作博弈力-位控制器中，仲裁参数 `alpha` 决定位置跟踪 Player 与力跟踪 Player 的相对权重：

```text
alpha -> 1: 偏位置跟踪、轨迹保持、RCM 几何安全
alpha -> 0: 偏力跟踪、接触力调节
```

旧方案使用 `|e_f|, K_hat, |e_r|` 作为模糊输入，其中 `K_hat` 是 RLS 估计的环境刚度。新方案改为使用交互力相对安全上下界的裕度，原因是安全风险首先来自当前交互力是否接近下界或上界，而不是刚度估计本身。

核心目标：

- 接触力接近上界时，增大 `alpha`，避免继续压入。
- 接触力接近下界时，减小 `alpha`，增强力控以避免脱离接触。
- 位置误差大时，增大 `alpha`，防止轨迹和 RCM 约束发散。
- 力在安全中部且位置误差小时，允许力-位平衡或偏力控修正。

## 2. 当前接入位置

默认策略名：

```text
force_margin
```

RCM 实验入口：

```python
python run_with_rcm.py --strategy force_margin
```

无 RCM 实验入口：

```python
python run_no_rcm.py --strategy force_margin
```

当前默认已改为 `force_margin`，因此不显式传 `--strategy` 时也会使用新方案。

旧策略仍保留用于对照：

```text
coop_fuzzy
fixed_08
fixed_05
fixed_02
```

## 3. 参数定义

### 3.1 RCM 模式

在 `run_with_rcm.py` 的 `Config` 中：

```python
F_desired = 0.5
F_min = 0.3
F_max = 1.0
```

含义：

- `F_desired = 0.5 N`: 期望接触反力。
- `F_min = 0.3 N`: 稳定接触下界，低于该值认为接触可能丢失。
- `F_max = 1.0 N`: 安全接触上界，高于该值认为存在过压风险。

### 3.2 无 RCM 模式

在 `run_no_rcm.py` 的 `Config` 中：

```python
F_desired = 2.0
F_min = 0.5
F_max = 3.0
```

无 RCM 扫描范围更大，期望力也更大，因此上下界使用更宽的力区间。

### 3.3 调度器内部参数

`ForceMarginFuzzyAlphaScheduler` 默认参数：

```python
k_upper = 0.35
k_lower = 0.25
alpha_min = 0.05
alpha_max = 0.95
upper_guard_alpha = 0.75
lower_guard_alpha = 0.25
smooth_tau = 0.08
```

含义：

- `k_upper`: 接近上界时提高 `alpha` 的修正强度。
- `k_lower`: 接近下界时降低 `alpha` 的修正强度。
- `alpha_min / alpha_max`: 安全修正后的 alpha 限幅。
- `upper_guard_alpha`: 已超过上界时，强制 `alpha_safe >= 0.75`。
- `lower_guard_alpha`: 已低于下界时，强制 `alpha_safe <= 0.25`。
- `smooth_tau`: alpha 一阶平滑时间常数，当前 `dt=0.01` 时平滑系数约为 `0.125`。

## 4. 输入变量

每个控制周期输入：

```text
F_norm    = abs(F_z)
F_desired = cfg.F_desired
F_min     = cfg.F_min
F_max     = cfg.F_max
e_r       = 位置误差范数
z_vel     = tool z 方向速度
```

当前代码已统一力误差符号：

```python
F_actual = abs(F_z)
e_f_scalar = F_actual - cfg.F_desired
```

解释：

- `e_f_scalar > 0`: 实际力大于期望力。
- `e_f_scalar < 0`: 实际力小于期望力。

这样安全边界修正中的方向参数与日志中的 `F_err` 保持一致。

## 5. 安全裕度计算

设：

```text
F = F_norm
Fd = F_desired
width = F_max - F_min
half_width = width / 2
```

到下界距离：

```text
d_lower = F - F_min
```

到上界距离：

```text
d_upper = F_max - F
```

归一化安全裕度：

```text
rho_F = clip(min(d_lower, d_upper) / half_width, 0, 1)
```

解释：

- `rho_F ~= 1`: 当前力位于安全区间中部。
- `rho_F ~= 0`: 当前力接近边界或已经越界。

边界方向参数：

```text
s_F = clip((F - Fd) / half_width, -1, 1)
```

解释：

- `s_F > 0`: 力偏大，靠近或超过上界，应增大 `alpha`。
- `s_F < 0`: 力偏小，靠近或低于下界，应减小 `alpha`。
- `s_F ~= 0`: 力接近期望值，不需要明显方向修正。

## 6. 模糊输入映射

新方案保留三输入模糊结构，但第二输入从环境刚度改为安全裕度。

### 6.1 力误差幅值 `F_h`

```text
F_h = clip(2 * abs(F - Fd) / (F_max - F_min), 0, 2)
```

论域：`[0, 2]`

模糊集：

| 标签 | 含义 | 参数 |
|---|---|---|
| PS | 力误差小 | `(0.0, 0.4, 0.8)` |
| PM | 力误差中 | `(0.4, 1.0, 1.5)` |
| PL | 力误差大 | `(1.2, 1.5, 2.0)` |

### 6.2 力安全裕度 `S_h`

```text
S_h = 6 * rho_F
```

论域：`[0, 6]`

模糊集：

| 标签 | 含义 | 参数 |
|---|---|---|
| PS | 安全裕度低，接近边界 | `(0.0, 1.0, 2.0)` |
| PM | 安全裕度中 | `(1.0, 3.0, 5.0)` |
| PL | 安全裕度高，位于安全区间中部 | `(3.0, 5.0, 6.0)` |

### 6.3 位置误差安全度 `D_r`

```text
D_r = clip(0.1 - abs(e_r) * (0.1 / 0.005), 0, 0.1)
```

解释：

- `e_r = 0`: `D_r = 0.1`，位置误差小。
- `e_r = 5 mm`: `D_r = 0`，位置误差大。

模糊集：

| 标签 | 含义 | 参数 |
|---|---|---|
| PS | 位置误差大 | `(0.0, 0.02, 0.04)` |
| PM | 位置误差中 | `(0.03, 0.05, 0.07)` |
| PL | 位置误差小 | `(0.06, 0.08, 0.1)` |

## 7. 模糊输出

新方案直接输出 `alpha_fuzzy`，不再使用旧方案中的 `lambda -> 1 - lambda` 语义。

输出论域：`[0, 1]`

| 标签 | 控制倾向 | 参数 |
|---|---|---|
| Z | 强力控 | `(0.0, 0.05, 0.1)` |
| PS | 偏力控 | `(0.05, 0.25, 0.5)` |
| PM | 力-位平衡 | `(0.25, 0.5, 0.7)` |
| P | 偏位控 | `(0.5, 0.7, 0.95)` |
| PL | 强位控/安全接管 | `(0.9, 0.95, 1.0)` |

## 8. 27 条主规则

输入顺序：

```text
(F_h, S_h, D_r)
```

输出：

```text
alpha_fuzzy
```

### 8.1 `F_h = PS`: 力误差小

| `S_h \ D_r` | PS: 位置误差大 | PM: 位置误差中 | PL: 位置误差小 |
|---|---:|---:|---:|
| PS: 安全裕度低 | PL | P | PM |
| PM: 安全裕度中 | PL | P | PM |
| PL: 安全裕度高 | P | PM | PM |

### 8.2 `F_h = PM`: 力误差中

| `S_h \ D_r` | PS: 位置误差大 | PM: 位置误差中 | PL: 位置误差小 |
|---|---:|---:|---:|
| PS: 安全裕度低 | PL | P | PS |
| PM: 安全裕度中 | P | PM | PS |
| PL: 安全裕度高 | P | PM | PS |

### 8.3 `F_h = PL`: 力误差大

| `S_h \ D_r` | PS: 位置误差大 | PM: 位置误差中 | PL: 位置误差小 |
|---|---:|---:|---:|
| PS: 安全裕度低 | PL | PM | Z |
| PM: 安全裕度中 | P | PS | Z |
| PL: 安全裕度高 | P | PS | Z |

规则解释：

- 位置误差大时，不允许 alpha 过低，避免轨迹和 RCM 发散。
- 力误差大且位置误差小时，允许强力控修正。
- 安全裕度低时，主规则只给基础 alpha，之后由 `s_F` 判断上界风险还是下界风险。

## 9. 安全方向修正

模糊输出记为：

```text
alpha_0 = alpha_fuzzy
```

风险强度：

```text
r_F = 1 - rho_F
```

安全修正：

```text
alpha_safe =
    alpha_0
    + k_upper * r_F * max(s_F, 0)
    - k_lower * r_F * max(-s_F, 0)
```

随后执行越界 guard：

```python
if F_norm >= F_max:
    alpha_safe = max(alpha_safe, upper_guard_alpha)
elif F_norm <= F_min:
    alpha_safe = min(alpha_safe, lower_guard_alpha)
```

该 guard 的原因：当力已经越界时，仅靠模糊规则和线性修正可能不够明确。超过上界必须给出位置/安全接管下限，低于下界必须给出力控保接触上限。

最后：

```text
alpha_safe = clip(alpha_safe, alpha_min, alpha_max)
```

## 10. 阶段先验融合

保留阶段检测：

```text
FREE_SPACE
APPROACHING
CONTACT_TRANSIENT
CONTACT_STEADY
RETREAT
```

当前新方案阶段参数：

| 阶段 | `alpha_phi` | `w_phi` | 解释 |
|---|---:|---:|---|
| FREE_SPACE | 1.00 | 1.00 | 自由空间纯位置控制 |
| APPROACHING | 0.85 | 0.60 | 接近阶段以位置下降为主 |
| CONTACT_TRANSIENT | 0.35 | 0.50 | 接触瞬态保留更多位置权重 |
| CONTACT_STEADY | 0.50 | 0.00 | 稳态接触完全交给安全裕度模糊仲裁 |
| RETREAT | 1.00 | 1.00 | 撤退阶段纯位置控制 |

融合公式：

```text
alpha_raw = w_phi * alpha_phi + (1 - w_phi) * alpha_safe
```

平滑：

```text
beta = dt / max(smooth_tau, dt)
alpha = alpha_prev + beta * (alpha_raw - alpha_prev)
```

当前 `dt=0.01, smooth_tau=0.08`，因此 `beta=0.125`。

## 11. 与控制律的关系

### 11.1 RCM 模式

RCM 模式中，新 alpha 只改变合作博弈增益查询权重，不改变 0428 已采用的 flange-space 控制结构：

```text
tool 期望轨迹
  -> RCM 正向映射到 flange 期望
  -> flange 空间位置/速度误差
  -> e_f, sigma_f
  -> K_eff(alpha, K_hat)
  -> u_flange
  -> J_flange.T @ wrench
```

即：几何层仍遵循当前 PDF 中的 RCM 正向映射方案，alpha 只负责力-位权重仲裁。

### 11.2 无 RCM 模式

无 RCM 模式仍直接在 tool 空间控制：

```text
e_r1 = x_tool - x_ref
e_r2 = xdot_tool - xdot_ref
e_f  = F_meas - F_des
sigma_f = leaky_integral(e_f)
u_tool = CooperativeGameController.compute_control(...)
tau = J_tool.T @ [u_tool, u_rot]
```

新 alpha 在该模式下同样根据力上下界安全裕度调整权重。

## 12. 当前测试结果

### 12.1 静态编译检查

命令：

```bash
python3 -m py_compile \
  tests/cooperative_gt_0428/src/alpha_scheduler_gt.py \
  tests/cooperative_gt_0428/run_with_rcm.py \
  tests/cooperative_gt_0428/run_no_rcm.py
```

结果：

```text
通过，无语法错误。
```

### 12.2 调度器数值 sanity check

测试设置：

```python
s = ForceMarginFuzzyAlphaScheduler(
    dt=0.01,
    F_min=0.3,
    F_max=1.0,
    F_desired=0.5,
    smooth_tau=0.01,
)
```

每个力值重复计算 120 步，使阶段进入 `CONTACT_STEADY` 并消除平滑初值影响。

测试结果：

| `F_norm` | 状态解释 | alpha | phase |
|---:|---|---:|---|
| 0.25 | 低于下界，接触可能丢失；由于也低于接触阈值，被阶段检测为自由空间 | 0.99 | FREE_SPACE |
| 0.31 | 刚高于下界，力偏小，增强力控保接触 | 0.2793 | CONTACT_STEADY |
| 0.50 | 接近期望力，力-位平衡 | 0.4827 | CONTACT_STEADY |
| 0.95 | 接近上界，适度提高位置/安全权重 | 0.5295 | CONTACT_STEADY |
| 1.20 | 超过上界，触发安全接管 guard | 0.75 | CONTACT_STEADY |

解释：

- `F=0.31` 时接近下界，`alpha` 降低，说明系统更偏力控以维持接触。
- `F=0.50` 时接近期望值，`alpha` 位于中等区间。
- `F=0.95` 时接近上界，`alpha` 相比中部略升高。
- `F=1.20` 时超过上界，`upper_guard_alpha=0.75` 生效，进入偏位置/安全接管。
- `F=0.25` 时小于 `F_min=0.3` 且低于阶段检测阈值 `0.3 N`，阶段检测判为自由空间，阶段先验强制纯位置控制。这与现有 PhaseDetector 设计一致；如果后续希望“低于下界但仍按接触丢失风险处理”，需要单独调整阶段检测阈值或新增接触保持状态。

## 13. 已知边界与后续建议

1. `F_min` 与 `PhaseDetector.F_thresh` 当前都在 0.3 N 附近。若 `F_norm < 0.3`，阶段会进入自由空间，安全裕度逻辑被阶段先验覆盖。

2. 当前 RCM 模式的 `F_min=0.3, F_max=1.0` 适合 `F_desired=0.5 N` 的小力扫描。真实实验时建议根据材料安全阈值和传感器噪声重新标定。

3. 当前无 RCM 模式的 `F_min=0.5, F_max=3.0` 对应 `F_desired=2.0 N`。若实际传感器噪声或接触材料变化较大，应优先调 `F_min/F_max`，而不是先调模糊规则。

4. `K_hat` 仍传入 `compute()` 以保持接口兼容，但新方案不使用它参与 alpha 仲裁。控制器本身仍使用 `K_hat` 查询合作博弈增益表。

5. 若需要更强的上界保护，可以提高 `upper_guard_alpha` 或 `k_upper`；若需要更强的保接触能力，可以降低 `lower_guard_alpha` 或提高 `k_lower`。
