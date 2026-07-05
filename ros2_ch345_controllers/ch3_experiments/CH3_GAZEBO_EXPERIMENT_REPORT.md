# 第三章 ROS2/Gazebo 控制器开发与可视化验证报告

生成日期：2026-07-02

## 1. 本轮目标

本轮目标是在不破坏已有稳定 `no_rcm` 控制器逻辑的前提下，新增第三章实验包 `/home/liu/franka_ros2_ws/src/ch3_experiments`，用于组织第三章所需的纯 Python 对比实验、Gazebo 在线实验、指标统计、Python 绘图和论文说明文档。

第三章实验边界保持为执行层力位协同控制：

```text
给定参考轨迹 + 给定期望接触力 + 给定柔性环境
  -> 执行层力位协同控制器
  -> 位置误差、力误差、力安全裕度、姿态稳定性、实时性指标
```

本章不引入第四章的人机参考仲裁，不使用 `alpha_HR`。

## 2. 本轮代码修改

新增 ROS2/Python 包：

```text
/home/liu/franka_ros2_ws/src/ch3_experiments
```

主要文件：

| 文件 | 作用 |
| --- | --- |
| `package.xml`, `setup.py`, `setup.cfg` | ROS2 `ament_python` 包定义和 console scripts |
| `config/ch3_default.yaml` | 纯 Python 第三章仿真默认参数 |
| `config/ch3_gazebo.yaml` | Gazebo/no-RCM 在线实验参数说明 |
| `data_types.py` | 第三章配置、柔性环境分区、接触状态和控制状态结构 |
| `flexible_surface.py` | 分段 Kelvin-Voigt 柔性接触环境 |
| `reference_trajectory.py` | 固定线性扫描参考轨迹 |
| `augmented_model.py` | 第三章力位增广模型表达 |
| `alpha_fp_analysis.py` | 力安全裕度与动态 `alpha_FP` 计算 |
| `controllers/*.py` | 阻抗、混合力位、固定 GT、动态 GT、QP/MPC 风格基线 |
| `ch3_contact_sim.py` | 纯 Python 第三章对比仿真 |
| `ch3_logger.py` | 统一 `.npz` 日志 |
| `ch3_metrics.py` | 复用 `no_rcm.analyze_ch3_result` 生成指标、图和报告 |
| `run_ch3_sim.py` | 纯 Python 批量实验入口 |
| `gazebo_no_rcm_adapter.py` | Gazebo/no-RCM 启动、运行、分析封装 |
| `run_ch3_gazebo_suite.py` | Gazebo 在线实验入口 |
| `CH3_EXPERIMENT_MANUAL.md` | 数学方法、代码与论文实验对应、使用说明 |

对已有稳定控制器的处理：

- 没有重写 `src/no_rcm/no_rcm/run_no_rcm.py` 的接近、扫描、复位主控制流程。
- 在线 Gazebo 控制仍调用 `no_rcm` 的 `run_no_rcm`、`no_rcm_effort_controller` 和 `analyze_ch3_result`。
- `ch3_experiments` 只负责实验组织、基线仿真、调用在线控制器和汇总分析。

## 3. 数学方法摘要

柔性环境使用分段 Kelvin-Voigt 模型：

```text
F_n = K_e(x) delta + B_e(x) max(0, -dot z)
delta = max(0, z_surface - z_tool)
```

第三章执行层增广状态为：

```text
xi = [e_p, e_v, e_f, sigma_f]^T
e_p = x - x_d
e_v = dx - dx_d
e_f = F - F_d
dot sigma_f = e_f - epsilon_f sigma_f
```

合作博弈折中目标：

```text
J(alpha_FP) = alpha_FP J_p + (1 - alpha_FP) J_f
```

其中：

```text
alpha_FP -> 1: 更偏位置跟踪
alpha_FP -> 0: 更偏力跟踪和力安全
```

在 no-RCM Gazebo 控制中采用分轴语义：

```text
alpha_x = 1
alpha_y = 1
alpha_z = alpha_FP
```

## 4. 纯 Python 对比实验验证

验证命令：

```bash
source /opt/ros/humble/setup.bash
source /home/liu/franka_ros2_ws/install/setup.bash

ros2 run ch3_experiments run_ch3_sim \
  --controllers impedance,hybrid,fixed_05,dynamic_gt,mpc_qp \
  --repeats 1 \
  --output-dir /home/liu/franka_ros2_ws/results/ch3_experiments_sim_verify
```

结果目录：

```text
/home/liu/franka_ros2_ws/results/ch3_experiments_sim_verify/ch3_sim_20260702_035230
```

报告：

```text
/home/liu/franka_ros2_ws/results/ch3_experiments_sim_verify/ch3_sim_20260702_035230/ch3_analysis/analysis/ch3_force_position_analysis_report.md
```

该实验验证了新增包的数据链路：控制器运行、`.npz` 记录、`.csv` 指标、`.png` 绘图和 `.md` 报告均能自动生成。

## 5. Gazebo 在线可视化验证

可视化验证命令：

```bash
source /opt/ros/humble/setup.bash
source /home/liu/franka_ros2_ws/install/setup.bash

ros2 run ch3_experiments run_ch3_gazebo_suite \
  --strategies fixed_05 \
  --trials 1 \
  --gui \
  --output-dir /home/liu/franka_ros2_ws/results/ch3_experiments_gazebo_gui_verify
```

Gazebo 启动日志：

```text
/home/liu/franka_ros2_ws/results/ch3_experiments_gazebo_gui_verify/gazebo_launch.log
```

结果目录：

```text
/home/liu/franka_ros2_ws/results/ch3_experiments_gazebo_gui_verify/no_rcm_20260702_035830
```

分析报告：

```text
/home/liu/franka_ros2_ws/results/ch3_experiments_gazebo_gui_verify/no_rcm_20260702_035830/ch3_analysis/analysis/ch3_force_position_analysis_report.md
```

Python 绘图结果：

![Gazebo metric bars](/home/liu/franka_ros2_ws/results/ch3_experiments_gazebo_gui_verify/no_rcm_20260702_035830/ch3_analysis/analysis/ch3_controller_metric_bars.png)

![Gazebo representative time series](/home/liu/franka_ros2_ws/results/ch3_experiments_gazebo_gui_verify/no_rcm_20260702_035830/ch3_analysis/analysis/ch3_representative_timeseries.png)

## 6. Gazebo 在线实验数据

本轮 GUI 可视化验证结果：

```text
策略：fixed_0.5
样本数：5673
扫描统计时长：56.72 s
力 RMSE：0.0877 N
峰值力误差：0.1839 N
力抖动：0.000549 N
力越界时间：0.0000 s
切向位置 RMSE：2.63 mm
法向位置 RMSE：0.23 mm
整体位置 RMSE：2.64 mm
平均姿态误差：1.17 deg
平均正面朝向误差：1.00 deg
平均计算时间：0.584 ms
最大计算时间：1.292 ms
复位最终位置误差：约 3.88 mm
复位最终姿态误差：约 1.62 deg
复位最终正面朝向误差：约 1.59 deg
```

判断：

- 力跟踪误差收敛到稳定小误差区间，未出现力安全越界。
- 扫描阶段位置误差稳定，未出现发散。
- 末端姿态和正面朝向误差保持在约 `1 deg` 量级。
- 复位阶段可回到初始笛卡尔位置附近，未出现此前的不稳定抬升或发散。
- 在线计算时间远小于 10 ms 控制周期，满足实时性要求。

## 7. 对比实验如何继续进行

固定优先级与动态优先级 Gazebo 对比：

```bash
ros2 run ch3_experiments run_ch3_gazebo_suite \
  --strategies fixed_02,fixed_05,fixed_08,continuous_force_margin \
  --trials 1 \
  --gui \
  --output-dir /home/liu/franka_ros2_ws/results/ch3_experiments_gazebo_compare
```

建议论文第三章正式结果按如下顺序生成：

1. 先运行纯 Python 多控制器多重复对比，筛选参数与解释趋势。
2. 再运行 Gazebo 单策略健康检查，确认接近、扫描、复位稳定。
3. 最后运行 Gazebo 策略组对比，比较固定优先级和动态优先级的力误差、位置误差、力越界时间和 `alpha_FP` 变化。

## 8. 当前结论

当前新增 `ch3_experiments` 包已经可以稳定完成第三章实验所需的核心闭环：

```text
实验配置 -> 控制器运行 -> 数据记录 -> 指标统计 -> Python 绘图 -> Markdown 报告
```

Gazebo GUI 可视化验证通过，当前 ROS2 no-RCM 控制器在 `fixed_05` 策略下满足误差收敛、姿态稳定、力安全和复位稳定要求。后续若要体现论文中动态 `alpha_FP` 的优势，应优先在 `continuous_force_margin` 策略上做对比实验，而不是改动已经稳定的接近、扫描和复位底层逻辑。
