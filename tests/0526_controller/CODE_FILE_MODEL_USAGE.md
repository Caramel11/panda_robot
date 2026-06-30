# 0526_controller 代码文件建模方法与使用说明

本文档按文件说明 `tests/0526_controller` 中各代码入口的建模思想、主要参数和使用方式。该目录是 0603 实验之前的 no-RCM 力位协同控制基线，主要用于验证虚拟柔性接触、在线刚度估计、合作博弈控制和真实力传感器接入流程。

## 总体模型

实验对象是 Franka Panda 末端在虚拟柔性表面上的法向接触扫描。虚拟环境使用 Kelvin-Voigt 形式：

$$
F_z = K_e(x)\delta + B_e(x)\dot{\delta},
$$

其中 $\delta=\max(0,z_s-z)$ 为压入深度，$K_e(x)$ 和 $B_e(x)$ 由分段刚度区域给出。控制目标被拆成位置目标、力目标和安全约束三部分。合作博弈控制器根据位置误差 $e_r$、力误差 $e_f$、估计刚度 $\hat K_e$ 和仲裁参数 $\alpha$ 生成任务空间控制量，再通过雅可比转成关节力矩。

## 顶层实验入口

| 文件 | 建模方法 | 使用方法 |
|---|---|---|
| `run_no_rcm.py` | Gazebo/虚拟接触 no-RCM 主入口。按 `Config` 定义扫描起点、扫描速度、目标力、虚拟刚度区间和控制周期；`run_trial()` 执行 home、approach、scan、retreat 阶段；力反馈既可来自虚拟环境，也可替换为外部传感器接口。 | 在 ROS1 Gazebo 环境中运行：`python3 tests/0526_controller/run_no_rcm.py --strategy continuous_force_margin --controller-mode pareto_iter`。用于快速调参数和保存 `.npz` 实验结果。 |
| `run_no_rcm_real.py` | 真实 Franka no-RCM 入口。保留虚拟环境估计器和合作博弈控制结构，但接触力优先从真实力传感器读取；阶段切换依赖接触阈值、力误差和扫描进度。 | 仅在真实机械臂已使能、急停和力传感器正常时运行。建议先用 Gazebo 或 mock sensor 复现同一参数。 |
| `no_rcm_ros2_real_node.py` | ROS2 节点版力位控制入口。`NoRCMForcePositionNode` 内部维护相位机、参考轨迹、在线刚度估计、alpha 调度器、力矩限幅和话题发布。模型上仍是 no-RCM 接触扫描，但执行结构更接近实时节点。 | 适合 ROS2 控制器桥接测试。先确认关节状态、机器人状态和力矩命令话题，再启动节点。 |
| `plot_experiment_analysis.py` | 实验结果分析入口。读取 workspace 级结果目录中的 `.npz`，计算力误差、位置误差、力抖动、alpha 轨迹和刚度估计等指标，生成图和表。 | `python3 tests/0526_controller/plot_experiment_analysis.py --input <result.npz>`；未指定时按脚本默认规则查找最近结果。 |

## 仲裁与估计文件

| 文件 | 建模方法 | 使用方法 |
|---|---|---|
| `fuzzy_logic.py` | 定义 `FuzzyLogicTool` 和 `Method2_IF_Else`。前者用三角/梯形隶属函数把人力、速度、距离或误差映射到仲裁量；后者提供规则表式 if-else 对照。 | 被 `alpha_scheduler_gt.py` 调用。单独调试时实例化 `FuzzyLogicTool(type="lambda_based")` 或 `type="delta_lambda_based"`，调用 `compute()` 查看输出。 |
| `kalman_filter.py` | 提供 `KalmanFilterFusion` 和 `DeltaLambdaUpdater`。模型假设仲裁参数或其增量是缓慢变化状态，通过预测-更新融合观测，降低模糊输出抖动。 | 与模糊推理串联使用。适合在离线脚本中输入一段 `delta_lambda` 或力/位置观测，检查输出是否平滑。 |
| `src/alpha_scheduler_gt.py` | alpha 调度核心。包含相位检测、连续相位先验、阶段感知模糊调度、force-margin 调度和 online-priority 调度。输入通常为 $F$、$e_f$、$\hat K_e$、$e_r$、$\dot z$，输出 $\alpha$。 | 主入口通过 `--strategy` 选择。调参时优先改 `F_min/F_max/F_desired`、相位阈值、平滑时间常数和 alpha 上下界。 |
| `src/env_estimator.py` | 在线环境估计器。根据接触力、压入深度和压入速度估计 $\hat K_e$、$\hat B_e$，并用低通/递推方式抑制噪声。 | 由主实验循环调用。调试时关注输入是否处于有效接触区，避免在 $\delta$ 太小或力传感器未稳定时更新刚度。 |
| `src/leaky_integrator.py` | 泄漏积分器。用于对持续误差进行积分补偿，同时通过泄漏项避免积分饱和。 | 被控制器或相位逻辑作为辅助模块调用。调参重点是积分增益、泄漏系数和输出限幅。 |

## 控制器与机器人接口

| 文件 | 建模方法 | 使用方法 |
|---|---|---|
| `src/gt_controller.py` | 合作博弈控制器。先为单轴质量-阻尼-刚度系统构造状态空间模型，再求解 ARE 或 Pareto 迭代增益表；运行时根据 $\alpha$ 和 $\hat K_e$ 插值增益，输出控制量。 | 主入口会预计算或读取增益表。开发时可单独运行 `precompute_gains()` 或 `precompute_pareto_gains()`，再用 `save_gains()` 缓存。 |
| `src/robot_interface.py` | ROS/Franka 任务空间接口。封装机器人状态读取、安全关节移动、RCM 几何、tool/flange 参考转换、姿态误差和 no-RCM/with-RCM 力矩计算。 | 由实验入口调用。真实机械臂调试时先验证 `update_robot_state()` 和雅可比维度，再发力矩。 |
| `src/utils.py` | 通用工具。包含 `VirtualStiffnessSurface`、ROS topic 力传感器包装、低通滤波、速率限制和 `DataLogger`。 | 虚拟环境用 `compute_force()`；结果记录用 `DataLogger.log()` 和 `save()`。 |
| `src/force_sensor_direct.py` | 串口/二进制力传感器直连接口。自动识别 28B/manual/vendor 数据帧，执行清零、启动流、单位换算和线程读取。 | 真实实验前单独实例化 `DirectForceSensorInput(port=...)`，确认 `available()`、`contact_force()` 和 `wrench_vector()` 有稳定输出。 |
| `src/__init__.py` | 包初始化文件。 | 无需直接运行。保证 `src` 目录可被 Python import。 |

## 推荐调试顺序

1. 先用 `run_no_rcm.py` 和虚拟力模型检查轨迹、阶段切换和保存结果。
2. 用 `plot_experiment_analysis.py` 查看力误差、位置误差和 alpha 曲线。
3. 若力估计抖动，优先检查 `env_estimator.py` 的有效接触门限和滤波时间常数。
4. 若控制输出突变，检查 `alpha_scheduler_gt.py` 的平滑项和 `utils.VectorRateLimiter`。
5. 真实实验只在 Gazebo 参数闭环稳定后运行 `run_no_rcm_real.py` 或 `no_rcm_ros2_real_node.py`。
