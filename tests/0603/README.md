# 0603 RCM / no-RCM controller interface alignment

本目录用于保留 2026-06-03 的 RCM 与 no-RCM 对照版本，所有文件均独立放在
`tests/0603/` 下，不修改原始 `0526_controller` 或 `cooperative_gt_0428`
目录。

## 文件说明

- `run_no_rcm.py`: 基于 `tests/0526_controller/run_no_rcm.py` 复制生成，默认
  `--strategy continuous_force_margin`，保留 0526 的 `--controller-mode`
  接口。
- `run_with_rcm.py`: 基于 `tests/cooperative_gt_0428/run_with_rcm.py` 复制生成，
  控制器接口参考 0526 更新为 `CooperativeGameController(control_mode=...)`，
  并新增 `--controller-mode {are,pareto_iter}`。RCM 几何、trocar 约束、
  flange-space 力矩装配、`error_rcm` 日志均保留。接近阶段采用限速参考:
  先慢速对齐扫描起点 x/y，再慢速下降，以降低起点接近过程中的速度峰值和
  RCM 误差。如果初始 tool z 已低于配置的 `scan_z`，脚本会自动使用当前
  高度下方的小距离作为局部接触阈值，避免一开始就误判进入扫描。接近段调试
  数据会保存到结果目录的 `approach_debug/` 子目录。
- `run_no_rcm_0526_reference.py`: 原始 0526 no-RCM 参考文件。
- `run_with_rcm_0428_reference.py`: 原始 cooperative_gt_0428 RCM 参考文件。
- `src/`: 从 0526 版本复制的控制器、调度器、估计器、机器人接口和数据记录依赖，
  包含 `pareto_iter` 和 `ContinuousForceMarginFuzzyAlphaScheduler`。
- `fuzzy_logic.py`, `kalman_filter.py`: `src/alpha_scheduler_gt.py` 运行所需的顶层
  依赖文件。
- `plot_latest_result.py`: 默认绘制最近一次实验 `.npz`，保存 overview 图和
  summary CSV。
- `plot_arbitration_compare.py`: 分析对照多组仲裁方法结果，按策略汇总 trial
  指标，保存对照图、时序叠加图和 CSV。

## 推荐命令

```bash
cd /home/liu/franka_ws_1101/src/panda_robot/tests/0603
python run_no_rcm.py --controller-mode pareto_iter
python run_with_rcm.py --controller-mode pareto_iter
```

如需关闭力传感器，仅使用虚拟环境回退运行 RCM 版本:

```bash
python run_with_rcm.py --controller-mode pareto_iter --no-force-sensor
```

默认策略均为 `continuous_force_margin`。也可以显式指定:

```bash
python run_with_rcm.py --strategy continuous_force_margin --controller-mode pareto_iter
python run_no_rcm.py --strategy continuous_force_margin --controller-mode pareto_iter
```

实验脚本默认会在结束后自动调用 `plot_latest_result.py`；当结果目录下存在多个
`.npz` 文件时，还会调用 `plot_arbitration_compare.py`。如只想保存图片不弹窗:

```bash
python run_with_rcm.py --controller-mode pareto_iter --plot-no-show
python run_no_rcm.py --controller-mode pareto_iter --plot-no-show
```

如需关闭自动绘图:

```bash
python run_with_rcm.py --controller-mode pareto_iter --no-auto-plot
```

单独绘制最近一次实验:

```bash
python plot_latest_result.py
python plot_latest_result.py --input /path/to/result_dir --no-show
```

多组仲裁方法对照:

```bash
python run_with_rcm.py --strategy all --trials 3 --controller-mode pareto_iter --plot-no-show
python plot_arbitration_compare.py --input /path/to/result_dir --no-show
```
