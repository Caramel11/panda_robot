# ROS1 0213 real_*.py 到 ROS2 shared_gt_controller 迁移说明

## 保留的控制逻辑

- 保留 `alpha_HR` 参考层人机仲裁思想：人类输入越强、持续越久，参考越偏向人侧修正。
- 保留 fuzzy/KF 融合结构：模糊输出直接仲裁量，增量通道经 `DeltaLambdaUpdater` 积分，再由二状态 Kalman 滤波平滑。
- 保留 GT 版本的 LQR/合作博弈形式：根据 `alpha_HR` 重新计算任务空间反馈增益。
- 保留 0213 with-RCM 几何含义：tool 参考先通过虚拟 trocar 点映射到 flange 参考，实际执行使用 flange Jacobian。
- 新增 no-RCM 版本：跳过 RCM 几何映射，直接使用 tool 参考、tool 误差和 tool Jacobian。

## 修改的接口

- `rospy + panda_robot.PandaArm` 改为 `rclpy + ros2_control effort command topic`。
- `PandaKinematics` 改为 Pinocchio frame Jacobian。
- ROS1 MoveIt 笛卡尔规划改为 Gazebo 中可重复的局部四段轨迹。
- Falcon 真实输入改为脚本化仿真人类输入，用于稳定复现实验。
- Excel 日志改为 `result.npz + summary.json`，并提供 Python 绘图入口。

## 稳定性处理

- 默认使用 no-gravity Gazebo world，并设置 `gravity_compensation_scale:=0.0`。
- 控制器使用力矩幅值限幅和力矩变化率限幅。
- with-RCM 启动时根据当前 tool-flange 轴线自动构造虚拟 trocar 点，避免绝对 trocar 与 Gazebo 初始位姿不一致。
- no-RCM 和 with-RCM 都以当前 Gazebo 位姿为局部任务起点，避免绝对目标越界。
- with-RCM 轨迹采用小幅 RCM 任务段：z 方向 12 mm、y 方向 18 mm、末端保持 6 mm 抬升；这是为了保持 trocar 约束下的可达性。
- no-RCM 直接执行自由空间四段轨迹，并额外加入 x 方向位移，体现无远心约束任务的自由跟踪能力。
- with-RCM 和 no-RCM 分别使用不同的 Gazebo effort 包络增益：with-RCM 更强调 RCM 几何约束下的阻尼，no-RCM 使用更硬的工具端位置环以消除自由空间稳态误差。

## 输出指标

每次实验保存：

- `tracking_error`: tool 到安全参考的误差。
- `rcm_error`: trocar 到 tool-flange 轴线距离，no-RCM 中为 0。
- `alpha`: 参考层人机仲裁参数。
- `F_h`: 脚本化人类输入强度。
- `tau`: 7 关节力矩命令。

成功判据采用后 25% 稳态窗口：

- no-RCM: `tracking_last_rms_m <= 0.003`。
- with-RCM: `tracking_last_rms_m <= 0.003` 且 `rcm_last_rms_m <= 0.003`。

## 本地 Gazebo 验证结果

验证日期：2026-07-02。仿真使用 `franka_gazebo_bringup` 的 no-gravity world，ROS2 effort controller 接口为 `/no_rcm_effort_controller/commands`。

| 模式 | 结果目录 | tracking 后 25% RMS | tracking 后 25% max | RCM 后 25% RMS | RCM 后 25% max | 结论 |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| with-RCM GT-KF | `/home/liu/franka_ros2_ws/results/shared_gt_gazebo/with_rcm_gt_kf_20260702_033236` | 2.40 mm | 2.74 mm | 1.37 mm | 1.57 mm | 通过 |
| no-RCM GT-KF | `/home/liu/franka_ros2_ws/results/shared_gt_gazebo/no_rcm_gt_kf_20260702_033424` | 2.12 mm | 2.20 mm | 0.00 mm | 0.00 mm | 通过 |

绘图文件：

- `/home/liu/franka_ros2_ws/results/shared_gt_gazebo/with_rcm_gt_kf_20260702_033236/shared_gt_result.png`
- `/home/liu/franka_ros2_ws/results/shared_gt_gazebo/no_rcm_gt_kf_20260702_033424/shared_gt_result.png`

迭代记录：

- 原始大幅 with-RCM 轨迹会在末段出现约 5.6 mm 工具端残差和约 3.1 mm RCM 残差。
- 并联工具端辅助力矩会破坏 RCM 几何映射，导致路径段出现明显振荡，因此最终移除。
- 最终版本回到 0213 的单一 RCM 映射结构，通过小幅 RCM 轨迹和按模式区分的 Gazebo effort 增益实现稳定收敛。
- no-RCM 初始自由空间位置环末段约 4.7 mm，提升工具端 PD 包络后进入 3 mm 以内。
