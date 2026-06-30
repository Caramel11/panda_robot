# 0630thesis 附属绘图代码方法与使用说明

本目录保存论文和投稿附件中使用的绘图、统计和模糊逻辑可视化脚本。代码主要来自第三章共享控制实验和第五章双仲裁实验的图表复现流程，目的是让论文插图可以从日志或脚本参数中重新生成。

## 1. 脚本分类

```text
attachments/plotting_code/
  fuzzy_logic.py             # 绝对/增量模糊推理定义
  kalman_filter.py           # 卡尔曼滤波辅助
  ieee_style.py              # IEEE 风格图像尺寸、字体和保存工具
  plot_fuzzy.py              # 绝对模糊规则图、隶属函数和曲面
  plot_delta_fuzzy.py        # 增量模糊规则图、隶属函数和曲面
  plot_fig3_ieee.py          # 一键重绘 Fig.3 的 IEEE 版本
  plot_gt_analysis.py        # 单次 GT 实验参数同步图
  plot_traj_gt.py            # 轨迹、力和事件图
  data_analysis_4method.py   # 四方法统计图
  data_analysis_GTvsMPC.py   # GT/MPC 对照统计
  data_analysis_OtherTarget.py# 其他目标场景统计
```

## 2. 方法说明

绘图代码服务于两类方法展示：

1. 人机共享控制仲裁：展示模糊输入、规则表、输出仲裁参数、主从轨迹和 RCM 误差。
2. 统计对比：从批量实验日志中提取误差、力、平滑性、轨迹 jerk 和目标距离，生成显著性分析图。

`fuzzy_logic.py` 提供两套推理：

- `lambda_based`：根据当前输入强度计算绝对仲裁参数。
- `delta_lambda_based`：根据误差变化趋势计算仲裁参数增量。

图像保存统一通过 `ieee_style.save_fig()`，通常同时输出 `.png` 和 `.pdf`。

## 3. 运行环境

推荐在工作空间根目录运行：

```bash
cd /home/liu/franka_ws_1101
python3 -m pip install numpy pandas matplotlib scipy seaborn openpyxl pillow
```

若服务器没有显示环境，设置：

```bash
export MPLBACKEND=Agg
export MPLCONFIGDIR=/tmp/matplotlib-codex
```

## 4. 生成模糊逻辑图

生成绝对模糊和增量模糊图：

```bash
cd /home/liu/franka_ws_1101/src/panda_robot/tests/0630thesis/attachments/plotting_code
python3 plot_fuzzy.py
python3 plot_delta_fuzzy.py
```

这些脚本默认把输出写入 `tests/0630thesis/figures` 或脚本指定的相对目录。若要生成 Fig.3 的合成图：

```bash
python3 plot_fig3_ieee.py --out fig3_ieee_output --surface-n 45
```

输出包括：

```text
fig3a_absolute_membership_ieee.png/pdf
fig3b_absolute_surface_ieee.png/pdf
fig3c_increment_membership_ieee.png/pdf
fig3d_increment_surface_ieee.png/pdf
fig3_combined_ieee.png/pdf
```

## 5. 生成批量统计图

四方法对照：

```bash
cd /home/liu/franka_ws_1101/src/panda_robot/tests/0630thesis/attachments/plotting_code
python3 data_analysis_4method.py
```

GT/MPC 对照：

```bash
python3 data_analysis_GTvsMPC.py
```

其他目标场景：

```bash
python3 data_analysis_OtherTarget.py
```

这些脚本会读取代码中配置的 `logs/.../summary_batch_results.xlsx` 或各 run 文件夹。若移动日志位置，需要先修改脚本顶部的 `INPUT_PATH`、`OUTPUT_DIR` 或相关路径常量。

## 6. 单次轨迹图

`plot_gt_analysis.py` 和 `plot_traj_gt.py` 用于单次实验细节图，常见输入文件包括：

- `force.xlsx`
- `error.xlsx`
- `arbitrary.xlsx`
- `end_torque.xlsx`
- `posterior_probabilities.xlsx`
- `file_traj_slave.xlsx`
- `file_traj_master.xlsx`
- `file_traj_robot.xlsx`
- `file_traj_human.xlsx`
- `file_traj_reshape.xlsx`

若某个文件缺失，脚本通常会跳过对应图或打印 warning。建议先确认 run 目录完整，再绘图。

## 7. 与论文目录的关系

`0630thesis` 是完整论文包，论文正文使用的图一般位于：

```text
tests/0630thesis/figures/
tests/0630thesis/figures/ch3_ch4/
tests/0630thesis/figures/ch5/
tests/0630thesis/figures/panda_robot/
```

附属绘图脚本不应直接修改正文 `.tex`。当图像更新后，需要检查：

1. 文件名是否与 `chapter*.tex` 中的 `\includegraphics` 一致。
2. 图像尺寸是否适合单栏或双栏位置。
3. 图注中的指标名称、方法名称和脚本输出是否一致。
4. PDF 编译是否仍能通过。

## 8. 版本管理建议

建议提交：

- 可复现实验脚本。
- 方法说明和使用说明文档。
- 论文需要长期保留的最终图像。

不建议提交：

- 临时日志目录。
- `.aux/.log/.toc/.synctex.gz/.xdv` 等 LaTeX 编译中间文件。
- `.npz/.npy/.zip` 等大体积结果包。
- 仅用于本机调试的缓存文件。
