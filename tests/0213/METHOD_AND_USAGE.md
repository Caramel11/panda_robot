# 0213 第三章共享控制实验方法与使用说明

`tests/0213` 保存第三章人机共享控制、微分博弈仲裁和模糊/卡尔曼意图估计相关实验代码。该目录包含两类脚本：一类用于实物或仿真实验采集日志，另一类用于从 Excel 日志生成统计表和论文图。

## 1. 方法背景

第三章关注刚性几何约束下的人机共享控制。机器人具有自主目标，操作者通过主端输入给出局部修正。系统不直接把人的瞬时输入作为底层控制命令，而是先估计人的监督意图，再用仲裁参数融合机器人参考和人侧参考。

典型方法组合：

- `GT_KF`：微分博弈控制 + 模糊/卡尔曼估计。
- `GT_Sigmoid`：微分博弈控制 + Sigmoid 仲裁。
- `MPC_KF`：MPC 控制 + 模糊/卡尔曼估计。
- `MPC_Sigmoid`：MPC 控制 + Sigmoid 仲裁。

论文中主要比较 RCM 误差、末端误差、交互力、力平滑性、轨迹 jerk 和障碍距离等指标。

## 2. 目录角色

```text
tests/0213/
  real_GT_KF.py                  # GT + Fuzzy/KF 实验入口
  real_GT_KF_with_other_target.py# 带目标切换/其他目标场景的 GT + KF
  real_GT_Sigmoid.py             # GT + Sigmoid 对照
  real_MPC_KF.py                 # MPC + KF 对照
  real_MPC_Sigmoid.py            # MPC + Sigmoid 对照
  fuzzy_logic.py                 # 模糊推理和输出去模糊化
  kalman_filter.py               # 卡尔曼滤波器
  lqr.py                         # LQR/二次型控制辅助
  analyze_ch3_for_thesis.py      # 当前推荐的第三章论文统计与绘图入口
  data_analysis*.py              # 旧版统计脚本
  plot*.py                       # 单次轨迹、模糊规则和系统图绘制脚本
```

日志默认来自：

```text
tests/0213/log_0213/
  0213_GT_KF/
  0213_GT_Sigmoid/
  0213_MPC_KF/
  0213_MPC_Sigmoid/
```

每次实验子目录通常包含：

- `error.xlsx`：RCM 误差、末端误差。
- `force.xlsx`：交互力。
- `file_traj_slave.xlsx`：从端轨迹。
- `file_traj_master.xlsx`：主端轨迹。
- `file_traj_robot.xlsx`：机器人自主参考。
- `file_traj_human.xlsx`：人侧参考。
- `file_traj_reshape.xlsx`：变形后轨迹。
- `arbitrary.xlsx`、`fuzzy.xlsx`、`FVD.xlsx`、`dFVD.xlsx`：仲裁和模糊推理相关中间量。

## 3. 推荐统计入口

从工作空间根目录运行：

```bash
cd /home/liu/franka_ws_1101
python3 src/panda_robot/tests/0213/analyze_ch3_for_thesis.py \
  --log-root src/panda_robot/tests/0213/log_0213 \
  --figure-dir src/panda_robot/tests/0630thesis/figures/ch3_ch4 \
  --output-dir src/panda_robot/tests/0213/ch3_thesis_outputs
```

如果需要保留所有原始 run，不按 IQR 规则剔除离群值：

```bash
python3 src/panda_robot/tests/0213/analyze_ch3_for_thesis.py \
  --no-filter-outliers
```

## 4. 统计流程

`analyze_ch3_for_thesis.py` 的处理步骤：

1. 按 `METHOD_DIRS` 遍历四类方法目录。
2. 从每个 run 的 Excel 日志读取误差、力和轨迹。
3. 计算 run 级指标：
   - `mean_error_rcm`
   - `mean_error_ee`
   - `mean_force_norm`
   - `Efs_force_smoothness`
   - `traj_rms_jerk`
   - `mean_dist_to_object`
4. 对核心指标执行 IQR 离群值标记。
5. 对方法间差异执行 Shapiro、Levene、t-test 或 Mann-Whitney 检验。
6. 输出 CSV、LaTeX 表和论文图。

## 5. 输出文件

默认输出到 `tests/0213/ch3_thesis_outputs`：

```text
ch3_run_metrics_raw.csv
ch3_run_metrics_with_outlier_flags.csv
ch3_run_metrics.csv
ch3_excluded_iqr_outliers.csv
ch3_pairwise_tests.csv
ch3_summary_stats.csv
ch3_summary_table.tex
```

默认论文图输出到 `tests/0630thesis/figures/ch3_ch4`：

```text
ch3_metrics_4method_box.png
ch3_representative_trajectory.png
ch3_representative_arbitration.png
ch3_significance_4method.png
```

## 6. 实物实验入口注意事项

`real_*.py` 会连接机器人或实验系统。运行前需要确认：

1. ROS master、Franka 控制器、主从端通信和力传感器均在线。
2. 日志根目录存在且有写权限。
3. 目标点、障碍位置和 RCM 几何参数与当前实验场景一致。
4. 主端输入范围和滤波器初值已经复位。

建议先用历史日志跑通 `analyze_ch3_for_thesis.py`，确认统计和绘图流程正常，再运行实物采集脚本。

## 7. 旧版脚本说明

- `data_analysis.py`：早期批量统计入口。
- `data_analysis_4method.py`：四方法对照统计。
- `data_analysis_GTvsMPC.py`：GT 与 MPC 对照。
- `data_analysis_OtherTarget.py`：目标切换或其他目标场景统计。
- `plot_traj_gt.py`：单次 GT 轨迹和事件图。
- `plot_fuzzy.py`、`plot_delta_fuzzy.py`：模糊隶属函数、规则表和曲面图。

旧版脚本中部分输入路径写死为 `logs/20260213/...` 或当前工作目录。复现实验时优先使用 `analyze_ch3_for_thesis.py`，需要旧图格式时再单独调用旧脚本。
