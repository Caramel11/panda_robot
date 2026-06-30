# 0603 代码文件建模方法与使用说明

`tests/0603` 是第四章力-位协同控制实验的主要目录，包含 no-RCM、with-RCM、真实入口、mock sensor、连续力裕度仲裁和实验绘图脚本。当前核心思路是：x/y 严格跟踪位置，z 方向由仲裁参数 $\alpha$ 在位置压入和接触力调节之间连续切换。

## 统一接触与控制模型

虚拟环境采用分段 Kelvin-Voigt 模型：

$$
F = K_e(x)\delta + B_e(x)\dot{\delta} + \epsilon,
\qquad
\delta=\max(0,z_s-z).
$$

`continuous_force_margin` 将目标力区间 $[F_{\min},F_{\max}]$、力误差 $e_f=F-F_d$、估计刚度 $\hat K_e$、位置误差 $e_r$ 和任务阶段一起映射为 z 向 $\alpha_z$。$\alpha_z$ 越小越偏向力，越大越偏向位置/几何约束。with-RCM 场景额外约束工具轴线通过 trocar 点。

## 主实验入口

| 文件 | 建模方法 | 使用方法 |
|---|---|---|
| `run_no_rcm.py` | no-RCM Gazebo/虚拟接触主入口。`Config` 定义目标力、扫描速度、刚度分区和连续仲裁参数；`apply_benchmark_config()` 可切换 default 与 force-margin challenge；`run_trial()` 执行接近、调整、扫描、退回并保存 `.npz`。 | 常用命令：`python3 tests/0603/run_no_rcm.py --strategy continuous_force_margin --controller-mode pareto_iter --benchmark force_margin_challenge --plot-no-show`。 |
| `run_with_rcm.py` | with-RCM Gazebo 主入口。模型在 no-RCM 基础上加入 trocar 点、工具长度、RCM 误差、姿态对齐和 RCM alpha floor；扫描过程同时满足接触力和远心约束。 | `python3 tests/0603/run_with_rcm.py --strategy continuous_force_margin --controller-mode pareto_iter --plot-no-show`。 |
| `run_no_rcm_force_margin_challenge.py` | no-RCM 压力测试副本。通过固定 z 参考、强刚度变化和较窄力裕度放大 fixed alpha 的缺陷，用来凸显 continuous force margin 的自适应优势。 | 用于论文对照或参数复查。运行方式与 `run_no_rcm.py` 类似，可批量比较 `fixed_08/fixed_05/fixed_02/continuous_force_margin`。 |
| `run_with_rcm_force_margin_challenge.py` | with-RCM 压力测试副本。除强刚度变化外，还引入 RCM 误差限幅、接近速度缩放、刚度相关 z 参考变化率等保护。 | 适合验证 RCM 场景下动态 $\alpha_z$ 是否同时改善力误差和 RCM 约束。 |
| `run_no_rcm_real.py` | 真实 Franka no-RCM 入口。真实力传感器优先，虚拟环境仅用于估计/安全辅助；阶段切换依赖接触阈值、调整窗口和扫描进度。 | 真实机械臂启用后运行。运行前必须确认工具坐标、力传感器方向、目标力区间和急停。 |
| `run_with_rcm_real.py` | 真实 Franka with-RCM 入口。将 RCM 几何和真实接触力接入同一合作博弈控制框架。 | 只建议在 mock sensor 和 Gazebo 已验证后使用。重点检查 trocar 点、工具长度和初始姿态。 |
| `run_no_rcm_0526_reference.py` | 从 0526 版本保留的 no-RCM 参考入口。 | 用于回溯旧参数和旧阶段逻辑，不作为当前论文主结果。 |
| `run_with_rcm_0428_reference.py` | 从 0428 版本保留的 with-RCM 参考入口。 | 用于对照早期 RCM 实现，不作为当前主入口。 |

## 调度器、估计器与控制器

| 文件 | 建模方法 | 使用方法 |
|---|---|---|
| `src/alpha_scheduler_gt.py` | alpha 仲裁核心。包含 `FixedAlphaScheduler`、阶段感知模糊调度、force-margin 模糊调度、`ContinuousForceMarginFuzzyAlphaScheduler` 和在线优先级调度。连续版本综合力安全裕度、刚度项、位置误差和相位先验。 | 主入口通过 `--strategy` 选择。调参优先看 `F_min/F_max/F_desired`、`alpha_min/alpha_max`、`smooth_tau`、刚度阈值和 force guard。 |
| `src/env_estimator.py` | 在线估计 $K_e,B_e$。用力、压入深度和压入速度做门限化递推估计，避免刚接触阶段把噪声写入刚度。 | 若 $\hat K_e$ 不随区域变化，检查有效接触门限、滤波时间常数和实际 `delta`。 |
| `src/gt_controller.py` | 合作博弈控制器。构造位置任务、力任务和安全力任务的二次型代价，使用 ARE 或 Pareto 迭代生成增益表，再对 $\alpha,\hat K_e$ 插值。 | `--controller-mode pareto_iter` 是当前推荐模式。若首次运行慢，说明正在预计算增益。 |
| `src/robot_interface.py` | Franka/Gazebo 接口。负责状态读取、RCM 几何、姿态误差、tool/flange 参考转换、任务空间控制到关节力矩映射、x/y/z 分轴 alpha 处理和日志字段。 | z-only 仲裁的实现主要在 no-RCM/with-RCM 力矩计算函数中。调试时检查记录字段 `alpha_x/y/z`。 |
| `src/utils.py` | 虚拟刚度表面、力传感器 topic 包装、滤波、速率限制和数据记录。 | 仿真用 `VirtualStiffnessSurface`；真实传感器 topic 用 `ForceSensorInput`；保存结果用 `DataLogger`。 |
| `src/force_sensor_direct.py` | 直接串口力传感器。处理帧格式识别、清零、启动流、单位换算和线程缓存。 | 真实实验前先单独确认传感器数据新鲜度和符号方向。 |
| `src/leaky_integrator.py` | 泄漏积分器。 | 用于慢变误差补偿，防止普通积分器在接触任务中饱和。 |
| `src/__init__.py` | 包初始化。 | 不直接运行。 |

## 绘图与验证脚本

| 文件 | 建模方法 | 使用方法 |
|---|---|---|
| `plot_latest_result.py` | 单次 `.npz` 概览分析。重构时间序列，计算力 RMS、峰值、位置误差、RCM 误差、alpha 和 $\hat K_e$，输出 overview 图和 CSV。 | `python3 tests/0603/plot_latest_result.py --input <result.npz> --no-show`。 |
| `plot_arbitration_compare.py` | 多策略对照分析。按目录收集不同策略 trial，计算均值/方差并画指标柱状图和时序叠加图。 | `python3 tests/0603/plot_arbitration_compare.py --input <result_dir> --no-show`。 |
| `test_real_mock_sensor_gazebo.py` | 用 mock force sensor 驱动真实入口脚本。`MockForceSensorInput` 按虚拟表面生成力，从而在 Gazebo 中测试 `_real.py` 的相位逻辑。 | `python3 tests/0603/test_real_mock_sensor_gazebo.py --mode no_rcm --strategy continuous_force_margin`。 |
| `fuzzy_logic.py` | 通用模糊推理工具，保留第三章共享控制中的绝对/增量仲裁逻辑。 | 被 alpha scheduler 或绘图脚本复用。 |
| `kalman_filter.py` | 仲裁参数平滑和 KF 融合工具。 | 用于降低模糊输出的跳变。 |

## 文档文件

| 文件 | 作用 |
|---|---|
| `README.md` | 简要说明 0603 实验入口和常用命令。 |
| `METHOD_AND_USAGE.md` | 面向第四章实验的详细方法与使用说明。 |
| `STIFFNESS_ESTIMATION_METHOD.md` | 环境刚度估计的数学描述。 |
| `continuous_force_margin_arbitration_report.md` | continuous force margin 仲裁方法和实验结果报告。 |
| `CODE_FILE_MODEL_USAGE.md` | 当前文件，按代码文件索引建模和使用方式。 |

## 推荐使用流程

1. 启动 Gazebo 和 Franka 控制器。
2. no-RCM 先跑 `run_no_rcm.py --benchmark force_margin_challenge`。
3. with-RCM 再跑 `run_with_rcm.py` 或 challenge 副本。
4. 用 `plot_arbitration_compare.py` 对比 `fixed_08/fixed_05/fixed_02/continuous_force_margin`。
5. 真实入口先用 `test_real_mock_sensor_gazebo.py` 验证，再接真实力传感器。
