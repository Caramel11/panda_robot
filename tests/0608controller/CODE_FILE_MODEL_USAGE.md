# 0608controller 代码文件建模方法与使用说明

`tests/0608controller` 保存第五章双层仲裁仿真。该目录将第三章的人机共享控制仲裁和第四章的力-位混合仲裁合并为一个可复现实验：参考层决定是否接受人类输入，执行层决定 z 向位置/力优先级。

## 双层仲裁模型

参考层仲裁参数为 $\alpha_{hr}$，用于生成期望参考：

$$
x_d = x_r + \alpha_{hr}\Delta x_h.
$$

执行层仲裁参数为 $\alpha_{fp}$，只作用于 z 向压入深度。$\alpha_{fp}\to1$ 表示位置优先，$\alpha_{fp}\to0$ 表示力优先。脚本用力安全裕度、估计刚度和切向任务误差共同计算动态 $\alpha_{fp}$。

## 代码文件

| 文件 | 建模方法 | 使用方法 |
|---|---|---|
| `run_ch5_dual_arbitration.py` | 自包含确定性仿真入口。`SimConfig` 给出扫描时间、目标力、刚度分区、噪声、时间常数和评分权重；`environment_stiffness()` 定义分段刚度；`human_delta()` 定义安全切向修正和危险法向压入；`compute_alpha_hr()` 根据人类输入方向和力裕度调节参考层；`compute_alpha_fp()` 根据力误差、风险裕度、刚度和切向误差调节执行层；`simulate_method()` 生成 fixed、hr_only、fp_only、dual_arbitration 等方法的完整轨迹。 | 推荐命令：`MPLCONFIGDIR=/tmp/matplotlib-codex python3 tests/0608controller/run_ch5_dual_arbitration.py --backend gazebo --tune --seed 11`。不需要 Gazebo 时可去掉 `--backend gazebo`。 |

## 结果文件和说明文档

| 文件 | 建模/数据含义 | 使用方法 |
|---|---|---|
| `README.md` | 说明主命令、最新有效结果目录和 Gazebo 探测状态。 | 作为复现实验的第一入口。 |
| `METHOD_AND_USAGE.md` | 详细描述第五章双仲裁方法、指标和参数。 | 写论文或改参数前先阅读。 |
| `CODE_FILE_MODEL_USAGE.md` | 当前文件，按代码文件解释建模和使用方式。 | 用于快速定位脚本功能。 |
| `results/ch5_dual_arbitration_20260608_215915/metrics.csv` | 各方法综合评分、力 RMS、力峰值、力抖动、越界时间、任务误差、人类输入接受率和法向压入抑制率。 | 可直接导入表格或绘图脚本。 |
| `results/ch5_dual_arbitration_20260608_215915/metrics.json` | 与 CSV 相同的结构化指标。 | 适合程序读取。 |
| `results/ch5_dual_arbitration_20260608_215915/tuning_history.csv` | 自动调参搜索历史，记录不同 `dual_stiffness_blend`、`dual_force_gain`、`dual_risk_gain` 等组合的得分。 | 用于解释最终参数来源。 |
| `results/ch5_dual_arbitration_20260608_215915/summary.md` | 最新实验结果摘要。 | 可作为论文结果段落的核对材料。 |
| `results/ch5_dual_arbitration_20260608_215915/*.png` | 指标柱状图、力/alpha/刚度时序、xy 轨迹、危险法向压入细节和 alpha-刚度响应图。 | 已提交的是轻量图表；原始 `.npz` 由 `.gitignore` 排除。 |

## 脚本内部函数索引

| 函数 | 作用 |
|---|---|
| `environment_stiffness(x)` | 构造低-高-低-高的分段刚度，用于检验 $\alpha_{fp}$ 对刚度变化的响应。 |
| `nominal_reference(t,cfg)` | 生成无人工干预时的扫描参考轨迹。 |
| `human_delta(t)` | 生成切向安全修正和法向危险压入。 |
| `force_margin(F,cfg)` | 计算力区间安全裕度 $\rho_F$ 和归一化力偏差。 |
| `project_reference(candidate,K_hat,cfg)` | 按估计刚度把参考 z 限制在安全力区间内。 |
| `compute_alpha_hr(...)` | 参考层仲裁，倾向接受切向输入、抑制危险法向输入。 |
| `compute_alpha_fp(...)` | 执行层仲裁，综合力误差、风险、刚度和切向误差。 |
| `simulate_method(method,cfg,seed)` | 对指定方法进行完整仿真。 |
| `metrics_for(result,cfg)` | 计算论文中的评价指标。 |
| `plot_results(...)` | 生成第五章结果图。 |
| `probe_gazebo(...)` | 检查本地 ROS/Gazebo 是否可达，不杀外部进程。 |

## 推荐使用流程

1. 修改 `SimConfig` 或 `human_delta()` 设定新实验场景。
2. 运行 `--tune` 搜索双仲裁参数。
3. 查看 `metrics.csv` 是否满足 dual_arbitration 优于 fixed 方法。
4. 将图表同步复制或引用到 `0630thesis/figures/ch5/`。
5. 若使用 Gazebo 探测，确认 `gazebo_status.json` 只作为本地诊断，不作为核心结果。
