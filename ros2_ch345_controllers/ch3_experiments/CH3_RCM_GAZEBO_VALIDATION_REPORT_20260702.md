# 第三章 RCM 约束控制器 Gazebo 验证报告

日期：2026-07-02

本报告记录本轮在本地 Gazebo 可视化仿真中对第三章 RCM 约束控制器的补全、调试和验证结果。实验基于 `/home/liu/franka_ros2_ws/src/ch3_experiments` 编排，在线控制器复用并完善 `/home/liu/franka_ros2_ws/src/ch3_controller/run_with_rcm.py`。

## 1. 本轮修改内容

### 1.1 末端朝向与自转抖动

历史 no-RCM 稳定版本表明，末端应采用固定笛卡尔姿态 `[-90, 0, -45] deg`，并使用笛卡尔空间 SO(3) 姿态控制，而不是用零空间或末端关节直接姿态保持去竞争扫描任务。本轮继续沿用该思路：

- `with_rcm_forward_ref` 在本地 Gazebo RCM 模式下由初始 flange 正面轴确定，避免末端正面偏到左前方 45 deg 或 90 deg。
- RCM 姿态环采用低通和限幅：`P_ori=10.0`、`D_ori=3.0`、`omega_filter_tau=0.06 s`、`omega_limit=1.2 rad/s`、`u_rot_limit=4.0`、`u_rot_rate_limit=24.0`。
- 每次 trial 开始重置姿态滤波状态，避免上一试次残留导致启动抖动。

同时，第四章/第五章共享控制节点已经参考第三章稳定控制器逻辑修复朝向和抖动；第三章 RCM 代码沿用相同姿态控制原则。

### 1.2 RCM 本地 Gazebo 初始化

原始 RCM 仿真中强制移动到固定 `INIT_JOINTS` 会在 effort/no-gravity 后端产生较大初始速度，破坏当前稳定 Gazebo 姿态。本轮在 `--local-rcm-task` 下改为：

- 使用当前稳定 Gazebo 初始姿态构造 tool、flange、trocar 几何。
- 跳过初始关节空间移动和最后关节空间回退。
- 将当前稳定 tool 高度作为虚拟接触平面，避免无意义下降卡住。
- 将 local 扫描长度限制为 10 mm，用于快速在线烟测。

### 1.3 RCM-aware 平移补偿

第一轮验证中，纯 RCM flange-space 控制姿态和 RCM 很稳，但 x 方向工具跟踪滞后明显，位置 RMSE 约 6.87 mm。直接加入较强 tool x/y 补偿后，位置 RMSE 降至约 1.33 mm，但 RCM 峰值升至 5.06 mm 并触发恢复超时。

最终保留的方案是 RCM-aware 小补偿：

```text
u_xy_boost = -s_rcm * (Kp_xy * e_xy + Kd_xy * de_xy)
```

其中：

```text
Kp_xy = 650
Kd_xy = 55
|u_xy_boost| <= 5 N
s_rcm = 1                       , e_rcm <= 2.5 mm
s_rcm linearly decays to 0      , 2.5 mm < e_rcm < 4.3 mm
s_rcm = 0                       , e_rcm >= 4.3 mm
```

该补偿只在 `--local-rcm-task` 下启用，不改变真实 RCM 任务和正式传感器实验的默认控制律。

## 2. 验证命令

构建：

```bash
source /opt/ros/humble/setup.bash
cd /home/liu/franka_ros2_ws
colcon build --packages-select ch3_controller ch3_experiments --symlink-install
source install/setup.bash
```

单策略稳定性验证：

```bash
ros2 run ch3_experiments run_ch3_rcm_gazebo_suite \
  --strategies fixed_05 \
  --trials 1 \
  --controller-mode pareto_iter \
  --output-dir /home/liu/franka_ros2_ws/results/ch3_rcm_orientation_verify8 \
  --timeout 120
```

多策略对比验证：

```bash
ros2 run ch3_experiments run_ch3_rcm_gazebo_suite \
  --strategies fixed_02,fixed_05,fixed_08,continuous_force_margin \
  --trials 1 \
  --controller-mode pareto_iter \
  --output-dir /home/liu/franka_ros2_ws/results/ch3_rcm_comparison_verify_20260702 \
  --timeout 120
```

多策略合并分析：

```bash
ros2 run ch3_experiments analyze_ch3_rcm_result \
  --input /home/liu/franka_ros2_ws/results/ch3_rcm_comparison_verify_20260702/combined_npz \
  --output-dir /home/liu/franka_ros2_ws/results/ch3_rcm_comparison_verify_20260702/combined_analysis
```

## 3. 单策略调试迭代结果

| 版本 | 主要设置 | force RMSE (N) | RCM RMSE (mm) | RCM peak (mm) | track RMSE (mm) | 姿态均值 (deg) | 结论 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| verify6 | 无 x/y boost | 0.110 | 0.535 | 0.634 | 6.875 | 1.009 | RCM 很稳，但 x 跟踪太慢 |
| verify7 | 强 x/y boost | 0.066 | 3.851 | 5.063 | 1.331 | 0.830 | 位置好，但触发 RCM 恢复超时 |
| verify8 | RCM-aware boost | 0.047 | 2.198 | 3.561 | 2.394 | 0.717 | 位置、力、RCM、姿态均衡，作为当前稳定版本 |

## 4. 多策略对比结果

统一分析目录：

```text
/home/liu/franka_ros2_ws/results/ch3_rcm_comparison_verify_20260702/combined_analysis/analysis
```

| strategy | force RMSE (N) | RCM RMSE (mm) | RCM peak (mm) | track RMSE (mm) | alpha mean | alpha min/max | 姿态均值 (deg) | front 均值 (deg) |
| --- | ---: | ---: | ---: | ---: | ---: | --- | ---: | ---: |
| fixed_0.2 | 0.0470 | 2.202 | 3.552 | 2.407 | 0.200 | 0.200 / 0.204 | 0.720 | 0.625 |
| fixed_0.5 | 0.0536 | 2.171 | 3.520 | 2.419 | 0.500 | 0.500 / 0.500 | 0.716 | 0.620 |
| fixed_0.8 | 0.0555 | 2.193 | 3.555 | 2.412 | 0.800 | 0.796 / 0.800 | 0.719 | 0.623 |
| continuous_force_margin | 0.0645 | 2.152 | 3.524 | 2.443 | 0.695 | 0.518 / 0.820 | 0.717 | 0.621 |

结论：

- 四个策略均在 Gazebo 本地可视化仿真中完整运行，未出现末端肉眼自转抖动，也未触发 RCM 恢复超时。
- RCM 峰值均约 3.52-3.56 mm，略高于 3.5 mm 软阈值但远低于 6.5 mm 暂停/恢复边界。
- 姿态误差均值约 0.72 deg，front-axis 误差均值约 0.62 deg，说明末端保持竖直向下且正面朝前。
- `continuous_force_margin` 的 alpha 在 0.518-0.820 间变化，可以体现自适应仲裁；固定 alpha 策略用于论文对照。
- 本地虚拟环境中固定低 alpha 的力 RMSE 最小，但差异较小；连续力裕度策略牺牲少量力误差以展示可解释的在线优先级调整。

## 5. 图表

![第三章 RCM 指标柱状图](figures/ch3_rcm_metric_bars_20260702.png)

![第三章 RCM 代表性时序图](figures/ch3_rcm_representative_timeseries_20260702.png)

## 6. 论文实验对应关系

第三章 RCM 相关实验建议按以下结构写：

1. RCM 几何约束实验：报告 `e_rcm`、`e_track`、姿态误差，证明工具轴稳定穿过 trocar 邻域。
2. 力位协同实验：报告 `F_measured` 与 `F_desired` 的 RMSE、峰值误差和抖动，证明接触力收敛。
3. 固定仲裁对比：`fixed_02/fixed_05/fixed_08` 展示不同力位权重下力误差、位置误差和 RCM 误差的 trade-off。
4. 自适应仲裁对比：`continuous_force_margin` 展示 alpha 随接触安全裕度变化，在接近风险边界时提高位置/RCM 优先级。
5. 姿态稳定性：报告 `ori_err_deg` 和 `front_axis_err_deg`，说明末端始终竖直向下且正面朝前。

## 7. 当前结论与后续建议

当前 ROS2/Gazebo RCM 控制器已经满足本地在线验证的阶段性要求：控制器可运行、误差有界收敛、末端姿态稳定、RCM 约束未触发暂停边界、数据与图表可用于第三章论文初稿。

后续若要进一步提高论文说服力，建议增加：

- 每个策略 3-5 次重复试验，报告均值和标准差。
- 分段刚度更明显的虚拟表面，让 `continuous_force_margin` 的 alpha 变化更有区分度。
- 更长扫描路径下的 RCM 约束极限测试。
- 将真实力传感器数据源接入同一分析流程，复用当前指标和绘图脚本。
