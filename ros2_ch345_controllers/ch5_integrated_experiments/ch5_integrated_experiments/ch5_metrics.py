#!/usr/bin/env python3
"""第5章综合实验指标计算模块。

本模块读取第4章/第5章控制器生成的 `result.npz` 与 `summary.json`，
重新计算论文第5章需要统一比较的指标。这样做的原因是：不同对比方法
可能来自同一个 Gazebo 控制器，但第5章论文需要以同一套判据评价
“第3章执行层 + 第4章参考层”的综合效果。

核心指标包括：

- `tracking_last_rms_m`：末段轨迹误差 RMS，用于判断是否收敛。
- `R_acc`：切向有益人类输入接受率，越大表示越能吸收操作者修正。
- `R_sup`：法向危险输入抑制率，越大表示越能抑制可能破坏力安全的输入。
- `T_vio_s`：力安全边界越界时间。
- `alpha_HR_*`：第4章参考层人机仲裁权重统计。
- `alpha_FP_*`：第3章执行层力/位置优先级统计。
- `score`：用于排序的紧凑综合分数，越小越好。
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from .data_types import vector3


METRIC_FIELDS = [
    "method",
    "task_mode",
    "scenario",
    "trial_id",
    "success",
    "tracking_last_rms_m",
    "tracking_last_max_m",
    "tracking_rms_m",
    "R_acc",
    "R_sup",
    "T_vio_s",
    "F_peak_N",
    "F_max_observed_N",
    "alpha_HR_mean",
    "alpha_HR_max",
    "S_alpha_HR",
    "alpha_FP_mean",
    "alpha_FP_min",
    "alpha_FP_max",
    "S_alpha_FP",
    "rcm_last_rms_m",
    "rcm_last_max_m",
    "target_tolerance_m",
    "score",
    "run_dir",
]


def _arr(data, key, default=None):
    """读取数组字段，并统一转换为浮点 `numpy` 数组。

    `result.npz` 中部分旧实验可能缺少新字段，例如早期数据没有
    `alpha_FP`。因此这里允许传入默认值，使指标脚本能兼容旧日志。
    """

    if key in data:
        return np.asarray(data[key], dtype=float)
    if default is None:
        raise KeyError(key)
    return np.asarray(default, dtype=float)


def _window_mask_from_signal(signal, threshold):
    """根据阈值选取有效实验窗口。

    第5章的 `R_acc` 和 `R_sup` 只应该在人类确实输入明显修正时计算。
    例如切向输入很小时，接受率分母接近零，此时计算比值没有意义。
    """

    return np.asarray(signal, dtype=float) > threshold


def compute_ch5_metrics(data, summary=None, method=None, trial_id=0):
    """计算单次第5章实验的统一指标。

    Args:
        data: `result.npz` 中的时序数据字典。
        summary: 控制器写出的 `summary.json`，用于补充任务模式等元数据。
        method: 第5章方法名；若为空，则从 summary 中回退读取。
        trial_id: 重复实验编号。

    Returns:
        一个普通字典，可直接写入 CSV。
    """

    summary = dict(summary or {})
    metrics = {}
    t = _arr(data, "t", np.arange(len(_arr(data, "tracking_error", []))) * 0.01)
    dt = float(np.mean(np.diff(t))) if len(t) > 1 else 0.01

    # 轨迹误差是判断控制器是否收敛的主指标。第5章使用最后 25%
    # 数据作为“末段”，避免初始运动阶段的瞬态误差掩盖最终收敛性能。
    tracking = _arr(data, "tracking_error")
    n0 = max(0, int(0.75 * len(tracking)))
    metrics["tracking_rms_m"] = float(np.sqrt(np.mean(tracking ** 2)))
    metrics["tracking_last_rms_m"] = float(np.sqrt(np.mean(tracking[n0:] ** 2)))
    metrics["tracking_last_max_m"] = float(np.max(tracking[n0:]))

    rcm = _arr(data, "rcm_error", np.zeros_like(tracking))
    metrics["rcm_last_rms_m"] = float(np.sqrt(np.mean(rcm[n0:] ** 2)))
    metrics["rcm_last_max_m"] = float(np.max(rcm[n0:]))

    try:
        # 参考层指标分解：
        # x_r 为自主/名义参考，x_h 为人类候选参考，x_d 为最终进入
        # 执行层的期望参考。将人类输入分解为切向和法向：
        # - 切向通常代表对路径的有益修正，应尽量接受；
        # - 法向可能对应接触力风险，应尽量抑制。
        x_r = vector3(data, "nominal_ref")
        x_h = vector3(data, "x_h")
        x_d = vector3(data, "tool_ref")
        n_down = np.array([0.0, 0.0, -1.0])
        delta_h = x_h - x_r
        delta_d = x_d - x_r
        h_n = delta_h @ n_down
        d_n = delta_d @ n_down
        h_t = delta_h - h_n[:, None] * n_down
        d_t = delta_d - d_n[:, None] * n_down
        tangential_window = _window_mask_from_signal(np.linalg.norm(h_t, axis=1), 0.001)
        normal_window = _window_mask_from_signal(h_n, 0.001)
        # R_acc = 最终切向参考幅值 / 人类候选切向幅值。
        # 值越大，表示系统越愿意接受切向有益输入。
        metrics["R_acc"] = float(
            np.sum(np.linalg.norm(d_t[tangential_window], axis=1))
            / (np.sum(np.linalg.norm(h_t[tangential_window], axis=1)) + 1e-9)
        ) if np.any(tangential_window) else 0.0
        # R_sup = 1 - 最终法向幅值 / 人类候选法向幅值。
        # 值越大，表示系统越能抑制法向危险输入。
        metrics["R_sup"] = float(
            1.0 - np.sum(np.abs(d_n[normal_window]))
            / (np.sum(np.abs(h_n[normal_window])) + 1e-9)
        ) if np.any(normal_window) else 0.0
    except KeyError:
        metrics["R_acc"] = 0.0
        metrics["R_sup"] = 0.0

    y_key = "y_f_realistic" if "y_f_realistic" in data else "y_f"
    if y_key in data:
        # `y_f_realistic` 是实物化仿真中的传感器法向力；没有该通道时回退到
        # Gazebo 原有力代理量 `y_f`。这里统计越界时间和峰值偏差。
        y_f = _arr(data, y_key)
        f_min = float(_arr(data, "F_min", [0.35])[0])
        f_max = float(_arr(data, "F_max", [1.2])[0])
        f_d = float(_arr(data, "F_d", [0.7])[0])
        metrics["T_vio_s"] = float(dt * np.sum((y_f < f_min) | (y_f > f_max)))
        metrics["F_peak_N"] = float(np.max(np.abs(y_f - f_d)))
        metrics["F_max_observed_N"] = float(np.max(y_f))
    else:
        metrics["T_vio_s"] = 0.0
        metrics["F_peak_N"] = 0.0
        metrics["F_max_observed_N"] = 0.0

    # alpha 是第4章参考层权重 alpha_HR。其均值反映人类参考总体参与程度，
    # S_alpha_HR 反映权重变化平滑性，数值过大通常意味着仲裁抖动。
    alpha = _arr(data, "alpha", np.zeros_like(tracking))
    metrics["alpha_HR_mean"] = float(np.mean(alpha))
    metrics["alpha_HR_max"] = float(np.max(alpha))
    metrics["S_alpha_HR"] = float(np.sqrt(np.mean((np.diff(alpha) / max(dt, 1e-9)) ** 2))) if len(alpha) > 1 else 0.0

    # alpha_FP 是第3章执行层力/位置优先级。第5章关心它是否随风险在线变化。
    alpha_fp = _arr(data, "alpha_FP", np.full_like(tracking, 0.5))
    metrics["alpha_FP_mean"] = float(np.mean(alpha_fp))
    metrics["alpha_FP_min"] = float(np.min(alpha_fp))
    metrics["alpha_FP_max"] = float(np.max(alpha_fp))
    metrics["S_alpha_FP"] = (
        float(np.sqrt(np.mean((np.diff(alpha_fp) / max(dt, 1e-9)) ** 2))) if len(alpha_fp) > 1 else 0.0
    )

    # Compact same-task score. It is intentionally monotone with bad outcomes:
    # lower is better. Coefficients match the Chapter 5 writing plan.
    metrics["score"] = float(
        0.20 * metrics["F_peak_N"]
        + 0.25 * metrics["T_vio_s"]
        + 60.0 * 0.20 * metrics["tracking_last_rms_m"]
        + 0.15 * max(0.0, 1.0 - metrics["R_acc"])
        + 0.15 * max(0.0, 1.0 - metrics["R_sup"])
        + 20.0 * 0.05 * metrics["rcm_last_rms_m"]
    )

    tolerance_m = float(summary.get("target_tolerance_m", 0.003))
    if "y_f_realistic" in data or "realistic_env_enabled" in data:
        tolerance_m = max(tolerance_m, 0.010)
    # The paper-level convergence requirement for realistic experiments is the
    # last-window RMS error.  Peak error is still recorded and plotted as a risk
    # indicator, but a short Gazebo impulse should not be classified as a failed
    # trial when RMS convergence, force safety and RCM RMS are all within bounds.
    success = (
        metrics["tracking_last_rms_m"] <= tolerance_m
        and metrics["T_vio_s"] <= 0.20
        and (summary.get("task_mode", "no_rcm") == "no_rcm" or metrics["rcm_last_rms_m"] <= tolerance_m)
    )
    metrics["success"] = bool(success)
    metrics["target_tolerance_m"] = tolerance_m
    metrics["method"] = method or summary.get("ch5_method", summary.get("arbitration_strategy", "unknown"))
    metrics["task_mode"] = summary.get("task_mode", "unknown")
    metrics["scenario"] = summary.get("scenario", "unknown")
    metrics["trial_id"] = int(trial_id)
    return metrics


def load_run(run_dir, method=None, trial_id=0):
    """读取一个原始实验目录并计算指标。

    原始实验目录应包含：
    - `result.npz`：所有时序变量；
    - `summary.json`：任务模式、策略、成功标记等摘要信息。
    """

    run_dir = Path(run_dir)
    with np.load(run_dir / "result.npz") as npz:
        data = {k: np.asarray(npz[k]) for k in npz.files}
    summary_path = run_dir / "summary.json"
    summary = {}
    if summary_path.exists():
        with open(summary_path, "r", encoding="utf-8") as f:
            summary = json.load(f)
    metrics = compute_ch5_metrics(data, summary, method=method, trial_id=trial_id)
    metrics["run_dir"] = str(run_dir)
    return metrics


def write_metrics_csv(rows, path):
    """把若干单次实验指标写成第5章统一 CSV 表格。"""

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=METRIC_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in METRIC_FIELDS})


def main(argv=None):
    """命令行入口：对一个或多个 run 目录计算指标。"""

    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", nargs="+", required=True)
    ap.add_argument("--output", required=True)
    args = ap.parse_args(argv)
    rows = [load_run(p) for p in args.runs]
    write_metrics_csv(rows, args.output)
    print(json.dumps({"runs": len(rows), "csv": args.output}, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
