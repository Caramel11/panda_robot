# ch3_experiments

`ch3_experiments` 是第三章“基于合作博弈的柔性接触力位协同控制方法”的实验组织包。

设计边界：

- 不替代已经调稳的 `src/no_rcm` 在线控制器。
- 纯 Python 仿真用于第三章批量对比、参数扫描、论文图表和机理解释。
- Gazebo 在线实验通过 `no_rcm` 后端执行，用于验证 ROS2/Gazebo 中接近、扫描、复位稳定性。
- RCM 约束 Gazebo 在线实验通过 `ch3_controller run_with_rcm` 与 `rcm_bringup` 执行，用于验证穿刺点误差、工具跟踪误差和力跟踪稳定性。
- 本包不引入第四章人机参考仲裁，不使用 `alpha_HR`。

常用命令：

```bash
source /opt/ros/humble/setup.bash
source /home/liu/franka_ros2_ws/install/setup.bash

ros2 run ch3_experiments run_ch3_sim --repeats 3
```

```bash
source /opt/ros/humble/setup.bash
source /home/liu/franka_ros2_ws/install/setup.bash

ros2 run ch3_experiments run_ch3_controller_baseline_comparison \
  --controllers standard_impedance,traditional_hybrid,standard_mpc,fixed_02,fixed_05,fixed_08,dynamic_gt \
  --repeats 3
```

```bash
source /opt/ros/humble/setup.bash
source /home/liu/franka_ros2_ws/install/setup.bash

ros2 run ch3_experiments run_ch3_smooth_letter_comparison \
  --run-gazebo \
  --methods standard_impedance,traditional_hybrid,standard_mpc,dynamic_gt \
  --scenarios ustc_smooth_u \
  --repeats 1
```

```bash
source /opt/ros/humble/setup.bash
source /home/liu/franka_ros2_ws/install/setup.bash

ros2 run ch3_experiments run_ch3_gazebo_suite \
  --strategies fixed_05 \
  --trials 1
```

```bash
source /opt/ros/humble/setup.bash
source /home/liu/franka_ros2_ws/install/setup.bash

ros2 run ch3_experiments run_ch3_rcm_gazebo_suite \
  --strategies fixed_05 \
  --trials 1
```

详细说明见 `CH3_EXPERIMENT_MANUAL.md` 和 `CH3_RCM_EXPERIMENT_MANUAL.md`。
