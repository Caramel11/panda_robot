# shared_gt_controller Gazebo 实验报告

## 目标

基于 `tests/0213/real_*.py` 的 ROS1 共享控制代码，生成 ROS2 控制器软件包，并分别验证：

- with-RCM: 保留论文中的远心点约束和人机参考层仲裁。
- no-RCM: 去掉远心点几何映射，保留共享控制、GT-KF 和安全参考融合逻辑。

判据为实验后 25% 稳态窗口中 tracking 误差收敛到 3 mm 以内；with-RCM 还要求 RCM 误差收敛到 3 mm 以内。

## ROS1 到 ROS2 的主要修改

- `rospy` 节点改为 `rclpy` 节点。
- `PandaArm.exec_torque_cmd` 改为发布 `Float64MultiArray` 到 `/no_rcm_effort_controller/commands`。
- `PandaKinematics` 改为 Pinocchio 读取 URDF 后计算 tool/flange pose、Jacobian 和重力项。
- ROS1 MoveIt 轨迹改为 Gazebo 可重复执行的局部四段任务轨迹。
- Falcon 真实人手输入改为脚本化 human delta，用于仿真中复现实验。
- Excel 数据记录改为 `result.npz`、`summary.json` 和 Python 绘图。

## 控制逻辑

with-RCM：

- 参考层仍计算 `alpha_HR`，并由 fuzzy/KF 或 sigmoid 版本决定人机参考融合权重。
- 工具端安全参考先通过虚拟 trocar 点映射成 flange 参考。
- 控制律在 flange 任务空间计算，再由 flange Jacobian 转成关节力矩。
- RCM 误差定义为 trocar 到 tool-flange 轴线的距离。

no-RCM：

- 跳过 trocar 映射。
- 直接以 tool pose、tool velocity 和 tool Jacobian 做任务空间控制。
- RCM 误差记为 0，用于和 with-RCM 的实验数据格式保持一致。

## 最终参数调整

- with-RCM 使用小幅 RCM 可达轨迹：12 mm z 向移动、18 mm y 向移动、末端 6 mm 抬升。
- with-RCM 提高 flange 位置环阻尼和位置包络，但不叠加工具端并联力矩，避免破坏远心几何。
- no-RCM 保留较大的自由空间轨迹，并提高工具端位置环增益以消除末段稳态误差。
- 两类任务都保留力矩幅值限幅和力矩变化率限幅。

## 验证命令

```bash
source /opt/ros/humble/setup.bash
source /home/liu/franka_ros2_ws/install/setup.bash

ros2 launch shared_gt_controller shared_gt_gazebo.launch.py \
  task_mode:=with_rcm controller_variant:=gt_kf

ros2 launch shared_gt_controller shared_gt_gazebo.launch.py \
  task_mode:=no_rcm controller_variant:=gt_kf
```

绘图：

```bash
ros2 run shared_gt_controller plot_shared_gt_result \
  --input /home/liu/franka_ros2_ws/results/shared_gt_gazebo/with_rcm_gt_kf_20260702_033236

ros2 run shared_gt_controller plot_shared_gt_result \
  --input /home/liu/franka_ros2_ws/results/shared_gt_gazebo/no_rcm_gt_kf_20260702_033424
```

## 实验结果

| 模式 | tracking 后 25% RMS | tracking 后 25% max | RCM 后 25% RMS | RCM 后 25% max | 结果 |
| --- | ---: | ---: | ---: | ---: | --- |
| with-RCM GT-KF | 2.40 mm | 2.74 mm | 1.37 mm | 1.57 mm | 通过 |
| no-RCM GT-KF | 2.12 mm | 2.20 mm | 0.00 mm | 0.00 mm | 通过 |

数据目录：

- `/home/liu/franka_ros2_ws/results/shared_gt_gazebo/with_rcm_gt_kf_20260702_033236`
- `/home/liu/franka_ros2_ws/results/shared_gt_gazebo/no_rcm_gt_kf_20260702_033424`

图像：

- `/home/liu/franka_ros2_ws/results/shared_gt_gazebo/with_rcm_gt_kf_20260702_033236/shared_gt_result.png`
- `/home/liu/franka_ros2_ws/results/shared_gt_gazebo/no_rcm_gt_kf_20260702_033424/shared_gt_result.png`
