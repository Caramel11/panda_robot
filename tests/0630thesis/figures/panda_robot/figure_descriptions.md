# Figure Descriptions

本文档对应本包 `figures/panda_robot` 下的 PNG 主图。PDF 文件为同名矢量副本，含义与 PNG 一致。

## 数据与统计口径

- 模糊控制相关图来自 `tests/files/fuzzy_logic.py`、`plot_fuzzy.py` 与 `plot_delta_fuzzy.py`。绝对通道使用 `F_h`、`T_h`、`D_r` 推理 `lambda`；增量通道使用 `Delta F_h`、`Delta T_h`、`Delta D_r` 推理 `Delta lambda`，两套规则均为 3 个输入、27 条规则。
- 目标切换局部时序图来自 `tests/0213/log_0213/0213_GT_KF_WithOtherTarget` 中两组原始 Excel 日志：`GT_KF_WithOtherTarget_20260214_015547` 和 `GT_KF_WithOtherTarget_20260214_064530`。图中力交互事件按 `|F_h| > 0.05 N` 检出。
- Fig.6、Fig.8、Fig.9 的统计柱状图使用 `tests/ieee_plot_all.py` 中当前固化的均值和标准差。Fig.8/Fig.9 也可从当前可见的 0213 物理日志按 `data_analysis_OtherTarget.py`、`data_analysis_4method.py` 的口径重算；当前 checkout 中可见样本数为 `GT_KF=29`、`GT_KF_WithOtherTarget=35`、`GT_Sigmoid=34`、`MPC_KF=45`、`MPC_Sigmoid=30`。

## 模糊推理结构图

### `fis_abs_mf.png`

该图展示绝对通道 FIS 的输入/输出隶属函数。实验设置中输入为交互力 `F_h in [0, 2] N`、交互持续时间 `T_h in [0, 6] s`、任务距离 `D_r in [0, 0.1] m`，输出为仲裁系数 `lambda in [0, 1]`。图中曲线直接来自 `lambda_based` 模糊集参数，用来说明人机权重随力、时间和距离变化的规则基础。

### `fis_abs_surface.png`

该图展示绝对通道 FIS 的三维控制曲面。生成时固定 `D_r = 0.03 m`，在 `F_h` 和 `T_h` 网格上调用 `FuzzyLogicTool.compute()` 计算 `lambda`。曲面体现了交互力和持续时间增大时，系统逐步提高人侧主导权的趋势。

### `fuzzy_rules_table.png`

该图给出绝对通道的 27 条模糊规则表。每条规则由 `F_h`、`T_h`、`D_r` 的低/中/高模糊标签组合映射到 `lambda` 的输出标签。它是 `fis_abs_mf.png` 和 `fis_abs_surface.png` 的规则来源，用于解释绝对权重如何从离散专家规则转为连续控制量。

### `fis_inc_mf.png`

该图展示增量通道 FIS 的输入/输出隶属函数。实验设置中输入为 `Delta F_h in [-3, 3] N/s`、`Delta T_h in [0, 2]`、`Delta D_r in [-0.08, 0.08]`，输出为 `Delta lambda in [-0.5, 0.5]`。该通道不直接给出权重，而是描述权重应当如何随交互趋势进行增减。

### `fis_inc_surface.png`

该图展示增量通道 FIS 的三维控制曲面。生成时固定 `Delta D_r = 0.0`，扫描 `Delta F_h` 与 `Delta T_h`，计算对应的 `Delta lambda`。曲面用于说明在交互增强或减弱时，权重更新方向和幅值的连续变化。

### `delta_fuzzy_rules_table.png`

该图给出增量通道的 27 条模糊规则表。规则输入为 `Delta F_h`、`Delta T_h`、`Delta D_r` 的负/零/正趋势标签，输出为 `Delta lambda` 的负大、负、零、正、正大标签。它补充说明增量权重更新不是手动调参曲线，而是由完整规则库驱动。

### `fig3_fis_ieee.png`

该图是论文排版用的 FIS 总览图，合并展示绝对通道和增量通道的隶属函数及控制曲面。它与单独的 `fis_abs_*`、`fis_inc_*` 图表达同一控制结构，但采用更紧凑的 IEEE 双栏布局。统计上该图不来自实验日志，而是由模糊集参数和脚本中的平滑示意曲面生成。

## 目标切换实验单次日志图

### `experiment_3d_trajectories_final_OtherTarget_false.png`

该图展示 `GT_KF_WithOtherTarget_20260214_015547` 的三维轨迹，叠加 Slave、Master、Robot、Human、Reshape、Direct 等轨迹流以及力交互起止点。当前图中目标点 a/b/c 的坐标为 `x=(0.25, 0.30, 0.35) m`、`y=0.06 m`、`z=0.03 m`。原始日志包含 2068 个样本、2 段力交互；平均交互力为 `0.317 N`，最大力为 `2.299 N`。

### `experiment_parameter_sync_analysis_OtherTarget_false.png`

该图对同一组 `OtherTarget_false` 日志进行同步时序分析，包含 `|F_h|` 与 `alpha/beta`、RCM/EE 误差、目标后验概率和末端扭矩。该组日志平均 RCM 误差为 `0.00339 m`，最大 RCM 误差为 `0.00950 m`；平均 EE 误差为 `0.00672 m`，最大 EE 误差为 `0.01739 m`。后验概率均值显示目标 b 占主导，`P_b` 平均约 `0.875`。

### `trajectory_xyz_with_mae_OtherTarget_false.png`

该图展示 `OtherTarget_false` 的 X/Y/Z 分量轨迹，并在每个轴上标注 Slave 相对 Master 的平均绝对误差。由原始 `file_traj_slave.xlsx` 与 `file_traj_master.xlsx` 重算得到 MAE：X 轴 `4.674 mm`、Y 轴 `2.109 mm`、Z 轴 `3.078 mm`。图中的竖线对应 `|F_h| > 0.05 N` 检出的力交互时段。

### `experiment_3d_trajectories_final_true.png`

该图展示 `GT_KF_WithOtherTarget_20260214_064530` 的三维轨迹，绘图元素与 `OtherTarget_false` 相同。当前图中目标点 a/b/c 的坐标为 `x=(0.25, 0.30, 0.35) m`、`y=0.00 m`、`z=0.03 m`。原始日志包含 4014 个样本、3 段力交互；平均交互力为 `0.114 N`，最大力为 `1.673 N`。

### `experiment_parameter_sync_analysis_true.png`

该图对 `true` 日志进行同步时序分析，展示力、仲裁权重、误差、意图后验和末端扭矩的联动关系。该组平均 RCM 误差为 `0.00326 m`，最大 RCM 误差为 `0.01027 m`；平均 EE 误差为 `0.00575 m`，最大 EE 误差为 `0.01423 m`。后验概率由目标 b 逐渐转向目标 c，最终 `P_c` 约为 `0.809`。

### `trajectory_xyz_with_mae_true.png`

该图展示 `true` 日志的 X/Y/Z 分量轨迹及 Slave-Master 逐轴 MAE。由原始轨迹重算得到 MAE：X 轴 `3.924 mm`、Y 轴 `2.286 mm`、Z 轴 `2.421 mm`。相较 `OtherTarget_false`，该组样本更多、交互力均值更低，但存在 3 段被检测出的力交互。

### `fig7_switching_ieee.png`

该图是上述六张目标切换相关面板的合成版，按两组物理实验日志对照展示三维轨迹、参数同步和三轴跟踪误差。上、下两组面板分别对应 `OtherTarget_false` 与 `true` 的实验条件，便于观察目标位置设置、意图后验变化和轨迹误差之间的关系。该图的定量依据来自两组原始 xlsx 日志，而不是手动绘制。

## 统计对比图

### `fig6_gt_mpc_ieee.png`

该图比较 GT+FIS 与 MPC+FIS 两类控制框架的仿真统计结果。脚本固化统计显示，GT+FIS 在平均 RCM 误差上为 `0.000362 +/- 0.000060 m`，MPC+FIS 为 `0.000727 +/- 0.000110 m`；平均 EE 误差分别为 `0.000602 +/- 0.000080 m` 与 `0.001014 +/- 0.000150 m`。同时 GT+FIS 的轨迹 RMS jerk 和控制力/力矩也更低，说明在该仿真设置下精度和平滑性均优于 MPC 基线。

### `fig8_other_target_ieee.png`

该图比较局部修正和目标切换两种物理实验条件。脚本固化统计中，目标不切换条件的平均 RCM/EE 误差为 `0.00303 +/- 0.00051 m`、`0.00525 +/- 0.00055 m`，目标切换条件为 `0.00320 +/- 0.00032 m`、`0.00591 +/- 0.00053 m`。该图说明目标切换会带来轻微跟踪误差和轨迹 jerk 增加，但平均障碍物距离保持在约 `0.054-0.055 m`。

### `fig9_four_method_ieee.png`

该图对 GT+FIS、GT+Sig.、MPC+FIS、MPC+Sig. 四种物理实验方法进行六指标对比。脚本固化统计显示，GT+FIS 的平均 RCM 误差为 `0.00303 +/- 0.00051 m`，EE 误差为 `0.00525 +/- 0.00055 m`，RMS jerk 为 `0.000402 +/- 0.000230`，均优于两种 MPC 方法。MPC+FIS 的平均交互力较低，但代价是 RCM/EE 误差和轨迹 jerk 明显增大。

### `fig9_four_method_ieee_column.png`

该图是 Fig.9 的单栏紧凑版本，只保留 RCM 误差、EE 误差和 RMS jerk 三个核心指标。数据来源与 `fig9_four_method_ieee.png` 相同，目的是在版面受限时突出精度和平滑性结论。三项指标中 GT+FIS 仍保持最小误差和最低 jerk。
