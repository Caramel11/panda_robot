# 第5章代码模块说明

本文档说明 `ch5_integrated_experiments` 包内各文件的用途、方法含义和使用场景。代码内已补充中文模块 docstring、函数注释和关键算法注释；本文档用于快速查阅整体结构。

## 1. 包定位

第5章包不是新的底层控制器包，而是综合实验包。它把：

- 第3章执行层：力/位置优先级 `alpha_FP`；
- 第4章参考层：人机参考仲裁 `alpha_HR`、安全投影 `x_h_safe`；
- Gazebo 在线实验；
- Falcon 实物输入；
- 统一日志、指标、图表、论文表格；

组织成可复现的实验流程。

## 2. Python 模块

| 文件 | 作用 | 主要输入 | 主要输出 |
|---|---|---|---|
| `method_switches.py` | 定义第5章对比方法到第3/4章控制器参数的映射 | 方法名，如 `full_method` | `arbitration_strategy`、`execution_strategy` 等 |
| `run_ch5_gazebo_suite.py` | 批量启动 Gazebo 在线实验 | task mode、scenario、methods、trials | `result.npz`、`summary.json`、CSV、PNG、报告 |
| `ch5_metrics.py` | 计算第5章统一评价指标 | 单次 run 目录 | `ch5_gazebo_metrics.csv` 行 |
| `ch5_plotting.py` | 生成论文分析图 | 指标 CSV、代表性 run | 指标柱状图、典型时序图 |
| `run_ch5_tvio_challenge.py` | 运行或 dry-run 第5章 `T_vio` 专项验证 | 方法列表、trials、是否启动 Gazebo | `T_vio` 指标 CSV、专项图、Markdown 报告 |
| `aggregate_results.py` | 汇总多次实验 | 多个 suite 或 CSV | 均值/标准差 CSV、LaTeX 表格、查漏报告 |
| `falcon_human_bridge.py` | Falcon 主端输入桥接 | `/falcon/ee_pose`、`/falcon/velocity`、`/falcon/joystick` | `/ch5/human_delta`、可选 `/falcon/force_cmd` |
| `hardware_preflight.py` | 实物实验前话题检查 | 需要检查的话题列表 | JSON 预检报告 |
| `generate_ch5_commands.py` | 打印常用命令 | 无 | Gazebo、聚合、Falcon、预检命令 |
| `data_types.py` | 通用数据结构和数组拼接工具 | `result.npz` 数据字典 | 三维向量矩阵 |

## 3. 对比方法含义

| method | 第4章参考层 | 第3章执行层 | 论文用途 |
|---|---|---|---|
| `direct_accept` | 直接接受人侧输入 | 固定 `alpha_FP=0.5` | 危险法向输入上界基线 |
| `balanced_fixed` | 固定 `alpha_HR=0.5` | 固定 `alpha_FP=0.5` | 固定权重基线 |
| `reference_only` | 动态 `alpha_HR` + 安全投影 | 固定 `alpha_FP=0.5` | 隔离第4章贡献 |
| `execution_only` | 自主参考，不接收人类修正 | 动态 `alpha_FP` | 隔离第3章贡献 |
| `no_projection` | 按人输入强度给权重，关闭安全投影与力裕度门控 | 动态 `alpha_FP` | 证明安全投影必要性，作为 T_vio 专项强消融基线 |
| `full_method` | 动态 `alpha_HR` + 安全投影 | 动态 `alpha_FP` | 第5章完整方法 |

## 4. 指标含义

| 指标 | 含义 | 趋势 |
|---|---|---|
| `tracking_last_rms_m` | 末段轨迹误差 RMS | 越小越好，理想仿真成功判据 `<= 0.003 m`，模拟实物实验判据 `<= 0.010 m` |
| `tracking_last_max_m` | 末段最大轨迹误差 | 越小越好，理想仿真成功判据 `<= 0.0045 m`，模拟实物实验判据 `<= 0.015 m` |
| `R_acc` | 切向有益输入接受率 | 越大表示越能吸收操作者修正 |
| `R_sup` | 法向危险输入抑制率 | 越大表示越能抑制危险输入 |
| `T_vio_s` | 力边界越界时间 | 越小越好，成功判据 `<= 0.20 s` |
| `F_peak_N` | 力代理量相对期望值的最大偏差 | 越小越好 |
| `alpha_HR_mean` | 第4章参考层人机权重均值 | 用于说明参考层是否参与 |
| `alpha_FP_mean` | 第3章执行层力/位置优先级均值 | 用于说明执行层是否参与 |
| `score` | 综合排序分数 | 越小越好，只用于同任务内比较 |

## 5. 典型使用流程

构建：

```bash
cd /home/liu/franka_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ch4_shared_gt_controller ch5_integrated_experiments --symlink-install
source install/setup.bash
```

运行完整对比：

```bash
ros2 run ch5_integrated_experiments run_ch5_gazebo_suite \
  --task-mode no_rcm \
  --scenario mixed_sequence \
  --methods balanced_fixed,reference_only,execution_only,no_projection,full_method \
  --trials 3 \
  --timeout-s 95
```

运行 `T_vio` 专项验证：

```bash
ros2 run ch5_integrated_experiments run_ch5_tvio_challenge \
  --run-gazebo \
  --methods direct_accept,balanced_fixed,no_projection,reference_only,full_method \
  --trials 3 \
  --task-duration-s 22 \
  --timeout-s 95
```

该脚本默认使用第4章新增的 `scenario:=tvio_stress_push`。工况包含持续危险法向压入和短暂切向恢复修正，目标是让直接接受、固定融合和无投影方法产生可观测 `T_vio`，同时检查完整方法是否保持 `T_vio<=0.20 s` 且末段轨迹误差收敛。

聚合结果：

```bash
ros2 run ch5_integrated_experiments ch5_aggregate_results \
  --inputs /home/liu/franka_ros2_ws/results/ch5_integrated \
  --output-dir /home/liu/franka_ros2_ws/results/ch5_integrated/aggregate \
  --expected-methods balanced_fixed,reference_only,execution_only,no_projection,full_method
```

Falcon 接入：

```bash
ros2 launch ros2_falcon falcon.launch.py
ros2 launch ch5_integrated_experiments ch5_falcon_bridge.launch.py
ros2 launch ch4_shared_gt_controller ch4_shared_gt_gazebo.launch.py \
  scenario:=external \
  arbitration_strategy:=full_method \
  execution_strategy:=dynamic_gt \
  external_human_topic:=/ch5/human_delta
```

## 6. 和论文第5章的对应关系

- “实验平台与系统结构”：引用 `falcon_human_bridge.py`、`hardware_preflight.py` 和第4章 launch。
- “综合控制链”：说明 `method_switches.py` 如何把第3章和第4章策略合并。
- “实验指标”：引用 `ch5_metrics.py` 中的指标定义。
- “Gazebo 在线实验”：引用 `run_ch5_gazebo_suite.py` 的批量实验流程。
- “结果图表”：引用 `ch5_plotting.py` 和 `aggregate_results.py` 输出。
- “消融实验”：对比 `balanced_fixed`、`reference_only`、`execution_only`、`no_projection`、`full_method`。
