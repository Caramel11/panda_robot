#!/usr/bin/env python3
"""第4章单次实验结果绘图脚本。

输入为某次 Gazebo 实验目录或其中的 ``result.npz``。脚本读取控制器记录的时序数据，
生成一张 3x2 综合图，用于检查：

1. 末端跟踪误差和 RCM 误差是否收敛。
2. ``alpha_HR``、``beta``、``kappa_N``、``rho_F`` 是否符合预期。
3. 人类输入和力代理量是否越界。
4. 候选参考、安全参考、最终参考之间的差异。
5. XY 平面轨迹和关节力矩范数是否异常。

该图主要用于论文实验前的调试和最终结果展示。
"""

import argparse
import json
import os
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def _load_input(path):
    """支持传入实验目录或直接传入 result.npz 文件。"""
    path = Path(path)
    if path.is_dir():
        npz = path / "result.npz"
    else:
        npz = path
    data = np.load(npz)
    return npz, data


def _arr(data, key, default=None):
    """从 NPZ 中读取数组；若字段缺失且给定默认值，则返回默认值。"""
    if key in data:
        return np.asarray(data[key])
    if default is None:
        raise KeyError(key)
    return np.asarray(default)


def main(argv=None):
    """命令行入口：读取数据、绘图并写出 plot_summary.json。"""
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True, help="Result directory or result.npz")
    ap.add_argument("--output", default=None, help="Output png path")
    args = ap.parse_args(argv)

    npz_path, data = _load_input(args.input)
    out = Path(args.output) if args.output else npz_path.parent / "ch4_shared_gt_result.png"

    t = _arr(data, "t")
    tracking = _arr(data, "tracking_error") * 1000.0
    rcm = _arr(data, "rcm_error", np.zeros_like(tracking)) * 1000.0
    alpha = _arr(data, "alpha", np.zeros_like(tracking))
    ori_err = _arr(data, "ori_err_deg", np.zeros_like(tracking))
    front_err = _arr(data, "front_axis_err_deg", np.zeros_like(tracking))
    beta = _arr(data, "beta", np.zeros_like(tracking))
    kappa = _arr(data, "kappa_N", np.ones_like(tracking))
    rho = _arr(data, "rho_F", np.ones_like(tracking))
    F_h = _arr(data, "F_h", np.zeros_like(tracking))
    y_f = _arr(data, "y_f_realistic", _arr(data, "y_f", np.zeros_like(tracking)))
    y_candidate = _arr(data, "y_f_candidate", y_f)
    f_min = _arr(data, "F_min", np.full_like(tracking, 0.35))
    f_d = _arr(data, "F_d", np.full_like(tracking, 0.7))
    f_max = _arr(data, "F_max", np.full_like(tracking, 1.2))
    tool = np.column_stack([_arr(data, f"tool_pos_{i}") for i in range(3)])
    ref = np.column_stack([_arr(data, f"tool_ref_{i}") for i in range(3)])
    nominal = np.column_stack([_arr(data, f"nominal_ref_{i}", ref[:, i]) for i in range(3)])
    x_h = np.column_stack([_arr(data, f"x_h_{i}", ref[:, i]) for i in range(3)])
    x_safe = np.column_stack([_arr(data, f"x_h_safe_{i}", ref[:, i]) for i in range(3)])
    tau_keys = [f"tau_{i}" for i in range(7)]
    tau_norm = np.zeros_like(t)
    if all(k in data for k in tau_keys):
        tau = np.column_stack([_arr(data, k) for k in tau_keys])
        tau_norm = np.linalg.norm(tau, axis=1)

    # 六个子图分别覆盖误差、仲裁权重、力代理量、参考分解、XY 轨迹和力矩范数。
    fig, axes = plt.subplots(3, 2, figsize=(12, 10), constrained_layout=True)
    ax = axes[0, 0]
    # 子图1：跟踪误差、RCM 误差和姿态正面朝前误差。
    threshold_mm = 10.0 if ("y_f_realistic" in data or "realistic_env_enabled" in data) else 3.0
    ax.plot(t, tracking, label="tracking")
    ax.plot(t, rcm, label="RCM")
    ax.axhline(threshold_mm, color="r", linestyle="--", linewidth=1, label=f"{threshold_mm:.0f} mm")
    ax.set_ylabel("error (mm)")
    ax.set_xlabel("time (s)")
    ax.grid(True, alpha=0.3)
    ax2 = ax.twinx()
    ax2.plot(t, ori_err, color="tab:green", alpha=0.65, label="ori")
    ax2.plot(t, front_err, color="tab:purple", alpha=0.65, label="front")
    ax2.set_ylabel("orientation (deg)")
    lines, labels = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax2.legend(lines + lines2, labels + labels2, fontsize=8)

    ax = axes[0, 1]
    # 子图2：第4章参考层关键权重和安全裕度。
    ax.plot(t, alpha, label="alpha_HR")
    ax.plot(t, beta, label="beta")
    ax.plot(t, kappa, label="kappa_N")
    ax.plot(t, rho, label="rho_F")
    ax.set_ylim(-0.05, 1.05)
    ax.set_xlabel("time (s)")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=2, fontsize=8)

    ax = axes[1, 0]
    # 子图3：人类输入强度和接触力代理量，黑色虚线/点线为安全边界和期望值。
    ax.plot(t, F_h, label="human input")
    if np.any(y_f):
        ax2 = ax.twinx()
        ax2.plot(t, y_candidate, color="tab:orange", alpha=0.45, label="F candidate")
        ax2.plot(t, y_f, color="tab:red", label="F final")
        ax2.plot(t, f_min, "k--", linewidth=0.8)
        ax2.plot(t, f_d, "k:", linewidth=0.8)
        ax2.plot(t, f_max, "k--", linewidth=0.8)
        ax2.set_ylabel("force proxy (N)")
        lines, labels = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax2.legend(lines + lines2, labels + labels2, fontsize=8)
    else:
        ax.legend()
    ax.set_xlabel("time (s)")
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    # 子图4：相对自主参考的切向/法向参考分量，用于观察投影和最终仲裁效果。
    ax.plot(t, (x_h[:, 1] - nominal[:, 1]) * 1000.0, label="candidate tangent y")
    ax.plot(t, (x_safe[:, 1] - nominal[:, 1]) * 1000.0, label="safe tangent y")
    ax.plot(t, (ref[:, 1] - nominal[:, 1]) * 1000.0, label="final tangent y")
    ax.plot(t, (nominal[:, 2] - x_h[:, 2]) * 1000.0, "--", label="candidate normal down")
    ax.plot(t, (nominal[:, 2] - ref[:, 2]) * 1000.0, "--", label="final normal down")
    ax.set_ylabel("reference offset (mm)")
    ax.set_xlabel("time (s)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes[2, 0]
    # 子图5：XY 平面轨迹，比较自主参考、人侧候选、安全参考、最终参考和实际工具轨迹。
    ax.plot(nominal[:, 0], nominal[:, 1], ":", label="x_r")
    ax.plot(x_h[:, 0], x_h[:, 1], "--", label="x_h")
    ax.plot(x_safe[:, 0], x_safe[:, 1], "-.", label="x_h_safe")
    ax.plot(ref[:, 0], ref[:, 1], label="x_d")
    ax.plot(tool[:, 0], tool[:, 1], color="k", linewidth=1.0, label="tool")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes[2, 1]
    # 子图6：关节力矩范数和人类输入，辅助检查控制是否出现异常尖峰。
    ax.plot(t, tau_norm, label="|tau|")
    ax.plot(t, F_h, label="human input")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("Nm")
    ax.grid(True, alpha=0.3)
    ax.legend()

    fig.suptitle(npz_path.parent.name)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=180)
    plt.close(fig)

    # 绘图摘要只给出快速检查指标；正式论文指标由 ch4_metrics.py 重算。
    n0 = max(0, int(0.75 * len(tracking)))
    summary = {
        "tracking_last_rms_mm": float(np.sqrt(np.mean(tracking[n0:] ** 2))),
        "tracking_last_max_mm": float(np.max(tracking[n0:])),
        "rcm_last_rms_mm": float(np.sqrt(np.mean(rcm[n0:] ** 2))),
        "rcm_last_max_mm": float(np.max(rcm[n0:])),
        "ori_last_rms_deg": float(np.sqrt(np.mean(ori_err[n0:] ** 2))) if len(ori_err) else 0.0,
        "front_last_rms_deg": float(np.sqrt(np.mean(front_err[n0:] ** 2))) if len(front_err) else 0.0,
        "front_last_max_deg": float(np.max(front_err[n0:])) if len(front_err) else 0.0,
        "force_peak_N": float(np.max(np.abs(y_f - f_d))) if np.any(y_f) else 0.0,
        "figure": str(out),
    }
    with open(npz_path.parent / "plot_summary.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
