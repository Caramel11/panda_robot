#!/usr/bin/env python3
"""第4章论文指标重算脚本。

在线控制器只负责把原始时间序列保存到 ``result.npz``。本模块根据论文中定义的
固定时间窗，从这些原始信号中重新计算表格指标。这样做的好处是：

1. 实时控制器不掺杂过多离线分析逻辑。
2. 指标定义调整后，可以直接重算旧实验数据，不需要重新跑 Gazebo。
3. 所有对比策略使用完全相同的窗口和公式，保证论文表格可复现。
"""

import argparse
import json
from pathlib import Path

import numpy as np


SCENARIO_WINDOWS = {
    "tangential_correction": {"tangent": [(4.0, 8.5)], "settle": [(18.0, 24.0)]},
    "unsafe_normal_push": {"normal": [(10.5, 14.0)], "settle": [(18.0, 24.0)]},
    "short_pulse_disturbance": {"pulse": [(9.5, 9.85)], "settle": [(18.0, 24.0)]},
    "sustained_intervention": {"sustained": [(14.2, 17.7)], "settle": [(18.0, 24.0)]},
    "mixed_sequence": {
        "tangent": [(4.0, 8.5)],
        "pulse": [(9.5, 9.85)],
        "normal": [(10.5, 14.0)],
        "sustained": [(14.2, 17.7)],
        "settle": [(18.0, 24.0)],
    },
    # U/S/T/C 圆滑字母轨迹总时长通常为 34 s。人类输入时间窗沿用
    # mixed_sequence，用于评价参考层仲裁；末段收敛窗口需要放在轨迹结束
    # 附近，否则 18-24 s 会落在字母描边中段，导致跟踪误差被误判为稳态误差。
    "ustc_smooth_u": {
        "tangent": [(6.8, 11.6)],
        "pulse": [(14.2, 15.5)],
        "normal": [(17.0, 21.8)],
        "sustained": [(23.8, 27.9)],
        "settle": [(30.0, 34.0)],
    },
    "ustc_smooth_s": {
        "tangent": [(6.8, 11.6)],
        "pulse": [(14.2, 15.5)],
        "normal": [(17.0, 21.8)],
        "sustained": [(23.8, 27.9)],
        "settle": [(30.0, 34.0)],
    },
    "ustc_smooth_t": {
        "tangent": [(6.8, 11.6)],
        "pulse": [(14.2, 15.5)],
        "normal": [(17.0, 21.8)],
        "sustained": [(23.8, 27.9)],
        "settle": [(30.0, 34.0)],
    },
    "ustc_smooth_c": {
        "tangent": [(6.8, 11.6)],
        "pulse": [(14.2, 15.5)],
        "normal": [(17.0, 21.8)],
        "sustained": [(23.8, 27.9)],
        "settle": [(30.0, 34.0)],
    },
}

# 在第4章柔性接触约定中，沿 N_DOWN 的正投影表示向下的危险法向压入。
N_DOWN = np.array([0.0, 0.0, -1.0])


def _arr(data, key, default=None):
    if key in data:
        return np.asarray(data[key], dtype=float)
    if default is None:
        raise KeyError(key)
    return np.asarray(default, dtype=float)


def _vec(data, prefix, fallback=None):
    cols = []
    for i in range(3):
        key = f"{prefix}_{i}"
        if key in data:
            cols.append(np.asarray(data[key], dtype=float))
        elif fallback is not None:
            cols.append(fallback[:, i])
        else:
            raise KeyError(key)
    return np.column_stack(cols)


def _mask(t, windows):
    mask = np.zeros_like(t, dtype=bool)
    for start, end in windows:
        mask |= (t >= start) & (t <= end)
    return mask


def _rms(x):
    x = np.asarray(x, dtype=float)
    if x.size == 0:
        return 0.0
    return float(np.sqrt(np.mean(x ** 2)))


def _safe_max(x):
    x = np.asarray(x, dtype=float)
    if x.size == 0:
        return 0.0
    return float(np.max(x))


def _reference_components(x_r, x_h, x_d):
    """把人类请求和最终参考分解为切向分量与法向分量。"""
    delta_h = x_h - x_r
    delta_d = x_d - x_r
    h_n = delta_h @ N_DOWN
    d_n = delta_d @ N_DOWN
    h_t = delta_h - h_n[:, None] * N_DOWN
    d_t = delta_d - d_n[:, None] * N_DOWN
    return h_t, d_t, h_n, d_n


def _arbitration_family(strategy):
    """把可能带 RCM 后缀的策略名归一到参考层仲裁方法族。"""
    if strategy is None:
        return ""
    s = str(strategy).strip().lower()
    if s.endswith("_rcm"):
        s = s[:-4]
    return s


def _normal_kept_by_strategy(strategy, raw, safe, alpha, kappa, observed_final):
    """按第4章参考生成逻辑重建最终保留的危险法向分量。"""
    s = _arbitration_family(strategy)
    raw = np.maximum(np.asarray(raw, dtype=float), 0.0)
    safe = np.maximum(np.asarray(safe, dtype=float), 0.0)
    alpha = np.clip(np.asarray(alpha, dtype=float), 0.0, 1.0)
    kappa = np.clip(np.asarray(kappa, dtype=float), 0.0, 1.0)
    observed_final = np.maximum(np.asarray(observed_final, dtype=float), 0.0)

    if s == "autonomous_only":
        return np.zeros_like(raw)
    if s == "direct_accept":
        return raw
    if s in {"fixed_blend", "single_sigmoid", "dynamic_no_projection", "dynamic_no_projection_open"}:
        return alpha * raw
    if s == "full_method":
        return alpha * kappa * safe
    return observed_final


def compute_metrics(data, scenario="mixed_sequence", tolerance_m=0.003, strategy=None):
    """计算第4章论文表格中报告的核心指标。

    参数：
      data: 已加载的 NPZ 对象或类似字典的数组映射。
      scenario: 场景名称，用于选择切向、法向、短扰动和稳态时间窗。
      tolerance_m: 稳态跟踪 RMS 成功阈值，默认理想仿真 3 mm；若数据包含
        实物化仿真通道，则自动放宽到 10 mm。

    关键输出：
      R_acc: 安全切向/持续人类输入的接受率。
      R_sup: 危险法向人类输入的抑制率。
      T_vio_s: 力代理量超出安全区间的总时间。
      S_alpha: alpha_HR 变化率 RMS，用作权重平滑性指标。
    """
    t = _arr(data, "t")
    tracking = _arr(data, "tracking_error", np.zeros_like(t))
    rcm = _arr(data, "rcm_error", np.zeros_like(t))
    ori_err = _arr(data, "ori_err_deg", np.zeros_like(t))
    front_err = _arr(data, "front_axis_err_deg", np.zeros_like(t))
    alpha = _arr(data, "alpha", np.zeros_like(t))
    if "y_f_realistic" in data or "realistic_env_enabled" in data:
        tolerance_m = max(float(tolerance_m), 0.010)
    windows = SCENARIO_WINDOWS.get(scenario, SCENARIO_WINDOWS["mixed_sequence"])
    settle_mask = _mask(t, windows.get("settle", []))
    if not np.any(settle_mask):
        settle_mask = np.arange(len(t)) >= int(0.75 * len(t))

    metrics = {
        "samples": int(len(t)),
        "tracking_rms_m": _rms(tracking),
        "tracking_max_m": _safe_max(tracking),
        "tracking_last_rms_m": _rms(tracking[settle_mask]),
        "tracking_last_max_m": _safe_max(tracking[settle_mask]),
        "rcm_rms_m": _rms(rcm),
        "rcm_max_m": _safe_max(rcm),
        "rcm_last_rms_m": _rms(rcm[settle_mask]),
        "rcm_last_max_m": _safe_max(rcm[settle_mask]),
        "ori_last_rms_deg": _rms(ori_err[settle_mask]),
        "ori_last_max_deg": _safe_max(ori_err[settle_mask]),
        "front_last_rms_deg": _rms(front_err[settle_mask]),
        "front_last_max_deg": _safe_max(front_err[settle_mask]),
        "alpha_mean": float(np.mean(alpha)) if alpha.size else 0.0,
        "alpha_max": _safe_max(alpha),
    }

    x_d = _vec(data, "tool_ref")
    x_r = _vec(data, "nominal_ref", fallback=x_d)
    x_h = _vec(data, "x_h", fallback=x_d)
    x_safe = _vec(data, "x_h_safe", fallback=x_h)
    h_t, d_t, h_n, d_n = _reference_components(x_r, x_h, x_d)
    _, safe_t, safe_h_n, safe_d_n = _reference_components(x_r, x_safe, x_d)
    normal_raw = _arr(data, "normal_raw", h_n)
    normal_safe = _arr(data, "normal_safe", safe_h_n)
    kappa_N = _arr(data, "kappa_N", np.ones_like(t))
    human_button = _arr(data, "human_button", np.zeros_like(t))
    human_event_type_id = _arr(data, "human_event_type_id", np.zeros_like(t))

    tangent_mask = _mask(t, windows.get("tangent", []) + windows.get("sustained", []))
    normal_mask = _mask(t, windows.get("normal", []))
    pulse_mask = _mask(t, windows.get("pulse", []))
    sustained_mask = _mask(t, windows.get("sustained", []))

    if np.any(tangent_mask):
        # 接受率：比较人类请求的切向位移中有多少进入了最终参考。
        req = np.sum(np.linalg.norm(h_t[tangent_mask], axis=1))
        acc = np.sum(np.linalg.norm(d_t[tangent_mask], axis=1))
        metrics["R_acc"] = float(acc / (req + 1e-9))
    else:
        metrics["R_acc"] = 0.0

    # 抑制率：比较危险法向请求和最终参考中剩余危险法向分量。
    # 对 34 s 正式实验优先使用论文预设 normal 时间窗；对 16 s quick-screen，
    # 预设窗口可能落在任务结束之后，因此退回到日志记录的危险法向交互事件。
    normal_threshold = 2.0e-4
    normal_eval_mask = normal_mask & (normal_raw > normal_threshold)
    eval_source = "window"
    req = np.sum(np.maximum(normal_raw[normal_eval_mask], 0.0))
    if req <= 1e-9:
        normal_eval_mask = (
            (human_event_type_id.astype(int) == 3)
            & (human_button > 0.5)
            & (normal_raw > normal_threshold)
        )
        eval_source = "event_type_3"
        req = np.sum(np.maximum(normal_raw[normal_eval_mask], 0.0))
    if req <= 1e-9:
        normal_eval_mask = (human_button > 0.5) & (normal_raw > normal_threshold)
        eval_source = "button_and_normal_raw"
        req = np.sum(np.maximum(normal_raw[normal_eval_mask], 0.0))

    if req > 1e-9:
        kept_series = _normal_kept_by_strategy(
            strategy,
            normal_raw[normal_eval_mask],
            normal_safe[normal_eval_mask],
            alpha[normal_eval_mask],
            kappa_N[normal_eval_mask],
            d_n[normal_eval_mask],
        )
        final = float(np.sum(kept_series))
        safe = float(np.sum(np.maximum(normal_safe[normal_eval_mask], 0.0)))
        metrics["R_sup"] = float(np.clip(1.0 - final / (req + 1e-9), 0.0, 1.0))
        metrics["R_project"] = float(np.clip(1.0 - safe / (req + 1e-9), 0.0, 1.0))
        metrics["R_sup_request_m"] = float(req)
        metrics["R_sup_kept_m"] = float(final)
        metrics["R_sup_mask_samples"] = int(np.sum(normal_eval_mask))
        metrics["R_sup_eval_source"] = eval_source
        normal_mask = normal_eval_mask
    else:
        metrics["R_sup"] = 0.0
        metrics["R_project"] = 0.0
        metrics["R_sup_request_m"] = 0.0
        metrics["R_sup_kept_m"] = 0.0
        metrics["R_sup_mask_samples"] = 0
        metrics["R_sup_eval_source"] = "none"

    y_f = _arr(data, "y_f_realistic", _arr(data, "y_f", np.zeros_like(t)))
    f_min = _arr(data, "F_min", np.full_like(t, 0.35))
    f_d = _arr(data, "F_d", np.full_like(t, 0.7))
    f_max = _arr(data, "F_max", np.full_like(t, 1.2))
    dt = float(np.mean(np.diff(t))) if len(t) > 1 else 0.01
    vio = (y_f < f_min) | (y_f > f_max)
    metrics["T_vio_s"] = float(dt * np.sum(vio))
    metrics["F_peak_N"] = float(np.max(np.abs(y_f - f_d))) if y_f.size else 0.0
    metrics["F_max_observed_N"] = _safe_max(y_f)
    if np.any(normal_mask):
        metrics["T_vio_normal_s"] = float(dt * np.sum(vio & normal_mask))
        metrics["F_peak_normal_N"] = float(np.max(np.abs(y_f[normal_mask] - f_d[normal_mask])))

    if alpha.size > 1:
        # S_alpha 用于惩罚权重突变，尤其对应短时扰动实验。
        alpha_dot = np.diff(alpha) / max(dt, 1e-9)
        metrics["S_alpha"] = _rms(alpha_dot)
        if np.any(pulse_mask):
            metrics["alpha_peak_pulse"] = _safe_max(alpha[pulse_mask])
        if np.any(sustained_mask):
            metrics["alpha_mean_sustained"] = float(np.mean(alpha[sustained_mask]))

    ref_step = np.linalg.norm(np.diff(x_d, axis=0), axis=1) if len(x_d) > 1 else np.zeros(0)
    metrics["reference_jump_max_m"] = _safe_max(ref_step)
    metrics["safe_projection_delta_max_m"] = _safe_max(np.linalg.norm(x_safe - x_h, axis=1))
    metrics["target_tolerance_m"] = float(tolerance_m)
    metrics["success"] = bool(
        metrics["tracking_last_rms_m"] <= tolerance_m
        and metrics["tracking_last_max_m"] <= 1.5 * tolerance_m
        and metrics["rcm_last_rms_m"] <= tolerance_m
    )
    return metrics


def load_result(path):
    """读取实验输入文件，并转换为后续分析使用的数据结构。"""
    path = Path(path)
    if path.is_dir():
        path = path / "result.npz"
    return path, np.load(path)


def main(argv=None):
    """命令行入口，解析参数并执行本模块对应的实验、绘图或分析流程。"""
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True, help="Run directory or result.npz")
    ap.add_argument("--scenario", default=None)
    ap.add_argument("--output", default=None, help="Optional metrics json path")
    args = ap.parse_args(argv)

    npz_path, data = load_result(args.input)
    scenario = args.scenario
    strategy = None
    if scenario is None:
        summary_path = npz_path.parent / "summary.json"
        if summary_path.exists():
            with open(summary_path, "r", encoding="utf-8") as f:
                summary = json.load(f)
                scenario = summary.get("scenario", "mixed_sequence")
                strategy = summary.get("arbitration_strategy", summary.get("strategy"))
        else:
            scenario = "mixed_sequence"
    metrics = compute_metrics(data, scenario=scenario, strategy=strategy)
    out = Path(args.output) if args.output else npz_path.parent / "window_metrics.json"
    with open(out, "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)
    print(json.dumps({"metrics": str(out), **metrics}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
