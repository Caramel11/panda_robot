# 第四章 ROS2/Gazebo 实验报告

## 开发边界

本包从 `/home/liu/franka_ros2_ws/src/shared_gt_controller` 克隆得到，原包未继续修改。克隆包路径为：

```text
/home/liu/franka_ros2_ws/src/ch4_shared_gt_controller
```

本次只在克隆包中做最小改动：保留原有 ROS2 Gazebo 启动、Pinocchio 运动学、`no_rcm_effort_controller` effort 接口和 GT-KF 任务空间控制律；新增第四章参考层实验所需的人类输入序列、候选参考、安全投影、动态仲裁消融策略、数据字段、指标和绘图。

## 基于当前代码的修改

主要修改集中在 `ch4_shared_gt_controller/shared_gt_gazebo_node.py`：

- 新增参数 `arbitration_strategy`：支持 `full_method`、`direct_accept`、`fixed_blend`、`single_sigmoid`、`dynamic_no_projection`、`autonomous_only`。
- 新增参数 `scenario`：支持 `mixed_sequence`、`tangential_correction`、`unsafe_normal_push`、`short_pulse_disturbance`、`sustained_intervention`。
- 新增第4章变量记录：`x_tele`、`x_h`、`x_h_safe`、`x_d/tool_ref`、`beta`、`alpha_HR`、`kappa_N`、`rho_F`、`I_h`、`T_h`、`C_h`、`D_c`、`y_f`、`F_min`、`F_d`、`F_max`。
- 新增安全投影代理模型：根据候选参考的法向压入量预测接触力，并在安全区间内裁剪法向参考，同时保留切向参考。
- 新增第四章指标：切向接受率 `R_acc`、法向抑制率 `R_sup`、力越界时间 `T_vio_s`、力峰值 `F_peak_N`、权重平滑性 `S_alpha`。
- 调整 `mixed_sequence` 时间窗，使 18-24 s 作为收敛保持窗口，避免把正在变化的人类输入误计入稳态收敛评价。

绘图脚本 `plot_shared_gt_result.py` 增加了第4章变量展示，包括投影前后参考、接触力代理量、`alpha_HR/beta/kappa_N/rho_F` 和末端跟踪误差。

本次补充的实验辅助代码：

- `ch4_metrics.py`：从 `result.npz` 按论文实验窗口重算 `R_acc`、`R_sup`、`T_vio_s`、`F_peak_N`、`S_alpha`、参考跳变等指标，并写出 `window_metrics.json`。
- `run_ch4_gazebo_suite.py`：顺序启动 Gazebo 实验矩阵，监听 `Saved summary` 后自动结束当前仿真并进入下一组。
- `compare_ch4_results.py`：支持 `--recompute-window-metrics`，可直接从原始数据重算窗口指标后生成 CSV 和策略对比图。
- `export_ch4_paper_assets.py`：把论文需要引用的图、表、`result.npz`、`summary.json` 和 `window_metrics.json` 导出到固定资产目录。
- `CH4_EXPERIMENT_MANUAL.md`：补充了数学方法、实验场景、消融策略、批量运行、指标重算和论文资产导出说明。

## Gazebo 验证命令

```bash
source /opt/ros/humble/setup.bash
source /home/liu/franka_ros2_ws/install/setup.bash

ros2 launch ch4_shared_gt_controller ch4_shared_gt_gazebo.launch.py \
  task_mode:=no_rcm controller_variant:=gt_kf \
  arbitration_strategy:=full_method scenario:=mixed_sequence

ros2 launch ch4_shared_gt_controller ch4_shared_gt_gazebo.launch.py \
  task_mode:=no_rcm controller_variant:=gt_kf \
  arbitration_strategy:=direct_accept scenario:=mixed_sequence
```

绘图：

```bash
ros2 run ch4_shared_gt_controller plot_shared_gt_result \
  --input /home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo/no_rcm_gt_kf_full_method_mixed_sequence_20260702_035207

ros2 run ch4_shared_gt_controller plot_shared_gt_result \
  --input /home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo/no_rcm_gt_kf_direct_accept_mixed_sequence_20260702_034702
```

批量矩阵与窗口指标重算：

```bash
ros2 run ch4_shared_gt_controller run_ch4_gazebo_suite \
  --scenarios mixed_sequence \
  --strategies autonomous_only direct_accept fixed_blend single_sigmoid dynamic_no_projection full_method

ros2 run ch4_shared_gt_controller compare_ch4_results \
  --runs /home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo/no_rcm_gt_kf_full_method_mixed_sequence_20260702_035207 \
         /home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo/no_rcm_gt_kf_direct_accept_mixed_sequence_20260702_034702 \
  --recompute-window-metrics \
  --output-dir /home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo/comparison_20260702_window_metrics
```

## 实验结果

| 策略 | tracking 稳态 RMS | tracking 稳态 max | R_acc | R_sup | T_vio | F_peak | ref jump | 结论 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| full_method | 2.81 mm | 3.56 mm | 0.571 | 0.798 | 0.00 s | 0.068 N | 9.82 mm | 稳定通过 |
| direct_accept | 2.58 mm | 4.87 mm | 1.465 | -0.853 | 0.72 s | 0.560 N | 15.89 mm | 参考层未筛选导致力越界 |

`full_method` 的末段 RMS 小于 3 mm，且力越界时间为 0，满足第四章在固定执行层下验证参考层仲裁稳定性的要求。`direct_accept` 虽然末段 RMS 也较低，但危险法向输入直接进入参考层，产生 0.72 s 软安全区间越界，说明安全投影和法向抑制是必要的。

## 数据与图像

完整方法：

```text
/home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo/no_rcm_gt_kf_full_method_mixed_sequence_20260702_035207/result.npz
/home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo/no_rcm_gt_kf_full_method_mixed_sequence_20260702_035207/summary.json
/home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo/no_rcm_gt_kf_full_method_mixed_sequence_20260702_035207/window_metrics.json
/home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo/no_rcm_gt_kf_full_method_mixed_sequence_20260702_035207/ch4_shared_gt_result.png
```

直接接受基线：

```text
/home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo/no_rcm_gt_kf_direct_accept_mixed_sequence_20260702_034702/result.npz
/home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo/no_rcm_gt_kf_direct_accept_mixed_sequence_20260702_034702/summary.json
/home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo/no_rcm_gt_kf_direct_accept_mixed_sequence_20260702_034702/window_metrics.json
/home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo/no_rcm_gt_kf_direct_accept_mixed_sequence_20260702_034702/ch4_shared_gt_result.png
```

对比图表：

```text
/home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo/comparison_20260702_window_metrics/ch4_metrics_summary.csv
/home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo/comparison_20260702_window_metrics/ch4_strategy_comparison.png
```

论文资产导出目录：

```text
/home/liu/franka_ros2_ws/panda_robot_gt_controller_dev/tests/0630thesis/generated/ch4_assets/asset_manifest.json
/home/liu/franka_ros2_ws/panda_robot_gt_controller_dev/tests/0630thesis/generated/ch4_assets/figures/ch4_mixed_sequence_full_method_timeseries.png
/home/liu/franka_ros2_ws/panda_robot_gt_controller_dev/tests/0630thesis/generated/ch4_assets/figures/ch4_mixed_sequence_direct_accept_timeseries.png
/home/liu/franka_ros2_ws/panda_robot_gt_controller_dev/tests/0630thesis/generated/ch4_assets/tables/ch4_metrics_summary.csv
```

## 结论

克隆包已经能够在本地 Gazebo 中稳定运行第四章 no-RCM 仲裁实验。完整方法在混合输入序列中保留切向修正、抑制危险法向压入，并在输入结束后的稳态窗口内把跟踪误差收敛到 3 mm 以内。对照基线显示，直接接受人类输入会带来明显接触力越界风险。
