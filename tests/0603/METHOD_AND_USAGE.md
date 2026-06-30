# 0603 RCM/no-RCM 力位仲裁实验方法与使用说明

本文档说明 `tests/0603` 下的 Gazebo、mock sensor 和真实入口实验代码。该目录对应论文第四章的柔性接触力-位协同控制实验，核心目标是比较固定仲裁参数 `fixed_08`、`fixed_05`、`fixed_02` 与动态仲裁策略 `continuous_force_margin` 的力误差、力抖动、位置误差、位置抖动和 RCM 约束保持效果。

## 1. 方法概览

实验对象是 Franka Panda 末端在分段 Kelvin-Voigt 虚拟环境上的接触扫描任务。接触法向力由压入深度和压入速度近似得到：

```text
F = K_e(x) * delta + B_e(x) * delta_dot + noise
```

其中 `K_e(x)` 和 `B_e(x)` 由 `Config.stiffness_zones` 给出，`delta` 是末端相对虚拟表面 `approach_z` 的压入量。no-RCM 实验只约束末端位置；with-RCM 实验还通过 trocar 点和工具长度建立固定点-工具轴线约束，使扫描同时满足接触力目标与远心运动约束。

控制器采用合作博弈式力-位反馈。`alpha` 是力位仲裁参数：

- `alpha` 越接近 1，越偏向位置/几何约束跟踪。
- `alpha` 越接近 0，越偏向接触力调节。
- 当前 z-only 仲裁版本中，`alpha` 只作用于 z 方向压入深度；x/y 方向严格跟踪扫描位置。

动态策略 `continuous_force_margin` 同时使用力误差、力安全裕度、估计刚度、位置误差和任务阶段信息。直观上，当接触力接近边界或环境刚度升高时，调度器降低 z 向位置优先级，使控制器更积极地调节接触力；当力处于安全区且位置误差较大时，调度器提高位置优先级，保证扫描轨迹质量。

## 2. 目录结构

```text
tests/0603/
  run_no_rcm.py                  # no-RCM Gazebo/虚拟接触主实验
  run_with_rcm.py                # with-RCM Gazebo/虚拟接触主实验
  run_no_rcm_real.py             # 真实 Franka no-RCM 入口
  run_with_rcm_real.py           # 真实 Franka with-RCM 入口
  test_real_mock_sensor_gazebo.py# 使用 mock force sensor 调真实入口的 Gazebo 测试
  plot_latest_result.py          # 单次 npz 结果概览图与 summary
  plot_arbitration_compare.py    # 多策略对照统计和时序图
  STIFFNESS_ESTIMATION_METHOD.md # 环境刚度估计方法说明
  src/
    alpha_scheduler_gt.py        # 固定/模糊/force-margin/连续仲裁策略
    env_estimator.py             # 在线刚度、阻尼估计
    gt_controller.py             # 合作博弈控制器和增益预计算
    robot_interface.py           # Franka/Gazebo 状态读取、力矩计算和数据记录
```

## 3. 主要实验入口

在运行前先启动 ROS/Gazebo，并加载 Franka 控制器：

```bash
cd /home/liu/franka_ws_1101
source devel/setup.bash
roslaunch panda_gazebo panda_world.launch \
  gui:=false use_custom_action_servers:=false start_moveit:=false load_gripper:=false
```

另开终端运行 no-RCM：

```bash
cd /home/liu/franka_ws_1101/src/panda_robot/tests/0603
source /home/liu/franka_ws_1101/devel/setup.bash
python3 run_no_rcm.py \
  --strategy continuous_force_margin \
  --controller-mode pareto_iter \
  --plot-no-show \
  --output-dir /home/liu/franka_ws_1101/results
```

运行 with-RCM：

```bash
cd /home/liu/franka_ws_1101/src/panda_robot/tests/0603
source /home/liu/franka_ws_1101/devel/setup.bash
python3 run_with_rcm.py \
  --strategy continuous_force_margin \
  --controller-mode pareto_iter \
  --plot-no-show \
  --output-dir /home/liu/franka_ws_1101/results
```

一次性跑固定参数和动态策略对照：

```bash
python3 run_no_rcm.py --strategy all --trials 3 --controller-mode pareto_iter --plot-no-show
python3 run_with_rcm.py --strategy all --trials 3 --controller-mode pareto_iter --plot-no-show
```

## 4. mock sensor Gazebo 测试入口

`test_real_mock_sensor_gazebo.py` 用 Gazebo 中的虚拟环境和 mock force sensor 调用 `_real.py` 入口，用来验证真实机械臂代码路径而不直接操作实物：

```bash
cd /home/liu/franka_ws_1101/src/panda_robot/tests/0603
source /home/liu/franka_ws_1101/devel/setup.bash
python3 test_real_mock_sensor_gazebo.py \
  --mode both \
  --strategy continuous_force_margin \
  --controller-mode pareto_iter \
  --trials 1 \
  --quick-scan-length 0.012 \
  --output-dir /home/liu/franka_ws_1101/results
```

常用参数：

- `--mode {no_rcm,with_rcm,both}`：选择测试 no-RCM、with-RCM 或两者。
- `--quick-surface/--no-quick-surface`：是否根据当前 tool z 快速设置虚拟表面。
- `--surface-offset`：虚拟表面相对当前 tool z 的偏移。
- `--force-noise-std`、`--force-bias`、`--force-scale`：mock force sensor 噪声、偏置和比例。
- `--quick-scan-length`：缩短扫描长度，适合快速调试。

输出目录形如：

```text
/home/liu/franka_ws_1101/results/mock_real_gazebo_YYYYMMDD_HHMMSS/
  mock_sensor_summary.csv
  mock_sensor_summary.json
  no_rcm/*.npz
  with_rcm/*.npz
  with_rcm/approach_debug/*.npz
```

## 5. 实验阶段设计

主实验分为三个阶段：

1. 接近阶段：固定 `alpha=1.0`，只做位置主导下探，避免未接触时力误差驱动控制器乱动。
2. 接触调整阶段：刚接触后的力和位置会有短时震荡。代码等待力误差、位置标准差和 RCM 误差进入稳定窗口，且不把这段数据写入正式扫描结果。
3. 正式扫描阶段：沿 x 方向匀速推进，z 方向由力位仲裁控制，记录 `F_measured`、`F_desired`、`alpha`、`K_hat`、位置误差、RCM 误差等指标。
4. 退回阶段：调度器进入 retreat 状态，控制器回到位置优先，并使用 `safe_move_to_joint_position` 回初始关节角。

## 6. 关键参数

no-RCM 入口的主要参数集中在 `run_no_rcm.py::Config`：

- `scan_start_x`, `scan_end_x`, `scan_y`：扫描线几何。
- `approach_z`：虚拟表面高度。
- `scan_vx`：正式扫描 x 方向速度。
- `F_desired`, `F_min`, `F_max`：目标力和安全边界。
- `stiffness_zones`：分段环境刚度和阻尼。
- `force_consistent_scan_z`：是否按 `F_desired / K_e` 生成物理一致 z 参考。
- `adjustment_*`：刚接触调整阶段的稳定判据。
- `continuous_*`：`continuous_force_margin` 的 alpha 上下界、平滑时间常数和力边界风险参数。

with-RCM 入口还包含：

- `trocar_position`：RCM 固定点。
- `tool_length`：flange 到 tool tip 的等效长度。
- `approach_xy_speed`, `approach_z_speed`：接近阶段分轴限速。
- `scan_rcm_recovery_*`：RCM 误差过大时的暂停、卸载和恢复逻辑。
- `scan_min_alpha_when_rcm_soft`：RCM 软约束区的 alpha 下限。

## 7. 输出与评价指标

每个 trial 保存一个 `.npz`：

```text
continuous_force_margin_t00.npz
fixed_08_t00.npz
fixed_05_t00.npz
fixed_02_t00.npz
```

常用字段：

- `t`：相对时间。
- `tool_position`、`x_ref`：实际位置和参考位置。
- `F_measured`、`F_desired`、`F_err`：接触力和力误差。
- `alpha`：力位仲裁参数。
- `K_hat`、`B_hat`：在线估计刚度和阻尼。
- `error_rcm`：with-RCM 的远心约束误差。
- `arbitration_strategy`：策略名称。

绘图命令：

```bash
python3 plot_latest_result.py --input /path/to/result_dir --no-show
python3 plot_arbitration_compare.py --input /path/to/result_dir --no-show
```

关注指标：

- `F_err_rms`：力跟踪均方根误差。
- `force_jitter`：力抖动，通常由差分或标准差描述。
- `pos_err_norm`：位置误差范数。
- `error_rcm_max`：RCM 最大约束误差。
- `alpha` 与 `K_hat` 的相关性：用于展示动态策略对刚度变化的响应。

## 8. 调参建议

- 若接触瞬间震荡较大，优先增大 `adjustment_min_time` 或放宽 `adjustment_timeout`，不要把瞬态段纳入正式统计。
- 若 `alpha` 对刚度不敏感，检查 `continuous_alpha_min/max`、刚度估计滤波和 `stiffness_zones` 差异是否足够明显。
- 若力误差小但位置误差大，适当提高 `continuous_safe_tracking_alpha` 或 `scan_vx` 降速。
- 若位置误差小但力越界，降低 `continuous_force_balance_alpha`，或缩小 `F_min/F_max` 使风险更早进入调度。
- with-RCM 若出现恢复模式频繁触发，优先降低 `scan_vx`，提高 `scan_rcm_recovery_alpha`，并检查 trocar/tool_length 是否与当前模型一致。

## 9. 安全说明

真实 Franka 入口 `run_no_rcm_real.py` 和 `run_with_rcm_real.py` 会向实物机械臂发送命令。运行前必须确认：

1. 急停、限位、外部力传感器和 ROS topic 均正常。
2. `INIT_JOINTS` 位姿附近无障碍物。
3. `F_desired/F_min/F_max` 与真实工具、工件和传感器量程一致。
4. 先用 `test_real_mock_sensor_gazebo.py` 和 Gazebo 验证同一参数组合。
