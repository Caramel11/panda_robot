# 第 3/4/5 章扩展数据记录验证记录（2026-07-05）

## 修改目的

为支持第 3、4、5 章对比实验、消融实验和真实环境噪声模拟实验的论文分析，本次在不改变控制律行为的前提下，将下列数据补充写入每次实验的 `result.npz` 和 `summary.json`：

- 执行层 `alpha_FP` 生成过程中的 force margin、跟踪误差、刚度估计和阶段先验等中间项；
- 位置、速度、力误差的切向/法向分解；
- 执行层位置项、速度项、力项、力误差积分项的控制输出分解；
- 任务空间输出限幅和执行层输出限幅状态；
- 任务力矩、重力力矩、真实环境扰动力矩、最终发送力矩及其分解；
- 真实环境模拟中的局部刚度、局部阻尼、软硬区坐标、法向压入量、六维力传感器噪声通道；
- 实验参数快照，包括任务时长、力边界、真实环境配置、人类输入配置和力矩限制。

## 涉及文件

- `src/ch3_experiments/ch3_experiments/no_rcm_force_position.py`
- `src/ch4_shared_gt_controller/ch4_shared_gt_controller/shared_gt_gazebo_node.py`
- `src/ch4_shared_gt_controller/CH345_EXTENDED_LOGGING_FIELDS.md`

其中 `no_rcm_force_position.py` 负责暴露第 3/5 章执行层内部项；`shared_gt_gazebo_node.py` 负责统一写入第 3/4/5 章 Gazebo 实验数据。

## 验证命令

```bash
source /opt/ros/humble/setup.bash
source /home/liu/franka_ros2_ws/install/setup.bash
ros2 run ch4_shared_gt_controller run_ch4_gazebo_suite \
  --strategies full_method \
  --scenarios stiffness_line \
  --repeats 1 \
  --task-mode no_rcm \
  --controller-variant gt_kf \
  --execution-strategy continuous_force_margin \
  --execution-alpha-profile stiffness_sensitive \
  --task-duration-s 3.0 \
  --timeout-s 65 \
  --settle-after-save-s 1.0 \
  --realistic-env \
  --realistic-profile sponge_300_600_two_block \
  --realistic-seed 20260705 \
  --realistic-tau-noise-gain 0.08 \
  --output-dir /home/liu/franka_ros2_ws/results/ch345_extended_logging_smoke_20260705_175728
```

## 验证结果

烟测结果目录：

```text
/home/liu/franka_ros2_ws/results/ch345_extended_logging_smoke_20260705_175728
```

关键文件：

```text
/home/liu/franka_ros2_ws/results/ch345_extended_logging_smoke_20260705_175728/no_rcm_gt_kf_full_method_continuous_force_margin_stiffness_line_20260705_175743/result.npz
/home/liu/franka_ros2_ws/results/ch345_extended_logging_smoke_20260705_175728/no_rcm_gt_kf_full_method_continuous_force_margin_stiffness_line_20260705_175743/summary.json
```

本次短时 Gazebo 运行共保存 301 个采样点、318 个字段。已确认下列新增字段存在并可读：

- `task_pos_err_0`、`task_force_error_N`、`task_tangential_pos_err_norm`
- `alpha_FP_after_stiffness`、`alpha_FP_stiffness_gate`、`alpha_FP_K_hat`
- `execution_component_force_2`、`execution_u_game_raw_2`、`execution_output_limit_active`
- `u_pos_pre_clamp_0`、`u_pos_limit_active`
- `tau_task_0`、`tau_gravity_0`、`tau_pre_realistic_0`、`tau_after_realistic_0`
- `tau_realistic_delta_0`、`tau_rate_clip_delta_0`
- `tau_pos_component_0`、`tau_rot_component_0`、`tau_rcm_component_0`
- `K_env_realistic`、`F_measured`、`actual_normal_down_m`、`sponge_local_y_actual_m`

烟测中的刚度记录能反映软硬区变化：`alpha_FP_K_hat` 从约 `300 N/m` 过渡到约 `598 N/m`，`alpha_FP_stiffness_gate` 从接近 `0` 过渡到接近 `1`，说明后续图像可以直接使用这些字段展示 `alpha_FP` 对刚度变化的响应过程。

## 检查结论

- `python3 -m py_compile` 通过。
- `colcon build --packages-select ch3_experiments ch4_shared_gt_controller --symlink-install` 通过。
- Gazebo 短时可视化烟测通过，结果成功保存。
- 扩展字段写入后未发现残留 ROS/Gazebo 进程。

后续完整第 3/4/5 章实验可复用同一记录格式；绘图脚本应优先读取 `CH345_EXTENDED_LOGGING_FIELDS.md` 中列出的字段，避免仅使用单一接触力曲线解释刚度切换或动态优先级变化。
