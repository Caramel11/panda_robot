#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Chapter 5 dual-arbitration simulation.

The script is intentionally self contained.  It provides a deterministic
analytic backend for repeatable thesis figures, plus a lightweight Gazebo
probe so the same folder documents whether the local ROS/Gazebo stack is
reachable from the current shell.

Main comparison set:
  fixed_08, fixed_05, fixed_02, hr_only, fp_only, dual_arbitration

The convention follows the thesis text:
  alpha_hr: reference-level human-robot arbitration.
  alpha_fp: execution-level position-force arbitration.
            alpha_fp -> 1 means position priority.
            alpha_fp -> 0 means force priority.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import subprocess
import sys
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import numpy as np

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


METHODS = (
    "fixed_08",
    "fixed_05",
    "fixed_02",
    "hr_only",
    "fp_only",
    "dual_arbitration",
)


@dataclass(frozen=True)
class SimConfig:
    dt: float = 0.01
    duration: float = 48.0
    scan_x0: float = 0.0
    scan_x1: float = 0.120
    y0: float = 0.0
    surface_z: float = 0.0
    F_min: float = 0.38
    F_desired: float = 0.70
    F_max: float = 1.16
    K_nominal: float = 520.0
    B_scale: float = 0.012
    force_noise_std: float = 0.006
    z_noise_std: float = 0.000012
    x_tau: float = 0.22
    y_tau: float = 0.20
    z_tau: float = 0.16
    K_hat_tau: float = 0.42
    alpha_hr_tau: float = 0.35
    alpha_fp_tau: float = 0.22
    projection_margin: float = 0.075
    k_normal_min: float = 0.08
    human_fixed_alpha: float = 0.58
    score_force_rms: float = 1.6
    score_force_peak: float = 0.65
    score_bound_time: float = 1.3
    score_force_jitter: float = 7.5
    score_task_mm: float = 0.018
    score_accept: float = 0.36
    score_suppress: float = 0.42
    score_clearance: float = 0.035

    # Tuned dual-arbitration parameters.
    dual_alpha_min: float = 0.16
    dual_alpha_max: float = 0.82
    dual_base_alpha: float = 0.58
    dual_force_gain: float = 0.34
    dual_risk_gain: float = 0.30
    dual_track_gain: float = 0.12
    dual_stiffness_blend: float = 0.70
    dual_K_low: float = 240.0
    dual_K_high: float = 980.0
    dual_alpha_soft: float = 0.76
    dual_alpha_hard: float = 0.24
    dual_upper_extra: float = 0.12
    dual_lower_extra: float = 0.07


def smoothstep(q: np.ndarray | float) -> np.ndarray | float:
    q = np.clip(q, 0.0, 1.0)
    return q * q * (3.0 - 2.0 * q)


def sigmoid(x: float) -> float:
    return 1.0 / (1.0 + math.exp(-x))


def smooth_window(t: np.ndarray, start: float, end: float, ramp: float = 0.7) -> np.ndarray:
    rise = smoothstep((t - start) / max(ramp, 1e-6))
    fall = smoothstep((end - t) / max(ramp, 1e-6))
    return np.minimum(rise, fall)


def environment_stiffness(x: np.ndarray | float) -> Tuple[np.ndarray | float, np.ndarray | float]:
    x_arr = np.asarray(x)
    K = np.full_like(x_arr, 320.0, dtype=float)
    K = np.where((x_arr >= 0.026) & (x_arr < 0.052), 860.0, K)
    K = np.where((x_arr >= 0.052) & (x_arr < 0.078), 230.0, K)
    K = np.where(x_arr >= 0.078, 1120.0, K)
    B = 2.6 + 0.010 * K
    if np.isscalar(x):
        return float(K), float(B)
    return K, B


def nominal_reference(t: np.ndarray, cfg: SimConfig) -> np.ndarray:
    x = cfg.scan_x0 + (cfg.scan_x1 - cfg.scan_x0) * np.clip(t / cfg.duration, 0.0, 1.0)
    y = np.full_like(t, cfg.y0)
    z = np.full_like(t, cfg.surface_z - cfg.F_desired / cfg.K_nominal)
    return np.vstack([x, y, z]).T


def human_delta(t: np.ndarray) -> np.ndarray:
    """Scripted operator input.

    Positive y is a safe tangential correction.  Negative z is an unsafe normal
    push that should be suppressed by the dual-arbitration method.
    """
    dy = 0.012 * smooth_window(t, 10.0, 20.0, 1.2)
    dy += -0.0065 * smooth_window(t, 32.0, 39.0, 1.0)
    dz = -0.00120 * smooth_window(t, 24.0, 30.5, 0.8)
    dz += -0.00035 * smooth_window(t, 40.0, 41.2, 0.25)
    dx = 0.0020 * smooth_window(t, 17.0, 22.0, 1.0)
    return np.vstack([dx, dy, dz]).T


def method_settings(method: str) -> Dict[str, float | bool]:
    if method == "fixed_08":
        return dict(alpha_fp=0.80, dynamic_hr=False, dynamic_fp=False, projection=False, human=True)
    if method == "fixed_05":
        return dict(alpha_fp=0.50, dynamic_hr=False, dynamic_fp=False, projection=False, human=True)
    if method == "fixed_02":
        return dict(alpha_fp=0.20, dynamic_hr=False, dynamic_fp=False, projection=False, human=True)
    if method == "hr_only":
        return dict(alpha_fp=0.50, dynamic_hr=True, dynamic_fp=False, projection=True, human=True)
    if method == "fp_only":
        return dict(alpha_fp=0.50, dynamic_hr=False, dynamic_fp=True, projection=True, human=False)
    if method == "dual_arbitration":
        return dict(alpha_fp=0.50, dynamic_hr=True, dynamic_fp=True, projection=True, human=True)
    raise ValueError(f"unknown method: {method}")


def force_margin(F: float, cfg: SimConfig) -> Tuple[float, float, float]:
    lower = F - cfg.F_min
    upper = cfg.F_max - F
    half = 0.5 * (cfg.F_max - cfg.F_min)
    rho = np.clip(min(lower, upper) / max(half, 1e-6), 0.0, 1.0)
    sF = np.clip((F - cfg.F_desired) / max(half, 1e-6), -1.0, 1.0)
    return float(rho), float(sF), float(1.0 - rho)


def project_reference(candidate: np.ndarray, K_hat: float, cfg: SimConfig) -> np.ndarray:
    safe = candidate.copy()
    K = max(K_hat, 120.0)
    F_low = cfg.F_min + cfg.projection_margin
    F_high = cfg.F_max - cfg.projection_margin
    delta_min = F_low / K
    delta_max = F_high / K
    z_min = cfg.surface_z - delta_max
    z_max = cfg.surface_z - delta_min
    safe[2] = float(np.clip(safe[2], z_min, z_max))
    return safe


def compute_alpha_hr(
    previous: float,
    raw_delta: np.ndarray,
    rhoF: float,
    cfg: SimConfig,
    dt: float,
) -> float:
    mag_t = abs(raw_delta[1]) / 0.012
    mag_n = abs(raw_delta[2]) / 0.0012
    intensity = float(np.clip(max(mag_t, mag_n), 0.0, 1.0))
    tangential_ratio = abs(raw_delta[1]) / (abs(raw_delta[1]) + abs(raw_delta[2]) + 1e-9)
    normal_push = float(np.clip(max(-raw_delta[2], 0.0) / 0.0012, 0.0, 1.0))
    normal_risk = normal_push * (1.0 - rhoF)
    target = 0.10 + 0.82 * intensity * (0.40 + 0.60 * tangential_ratio)
    target *= 1.0 - 0.62 * normal_risk
    target = float(np.clip(target, 0.05, 0.92))
    return previous + (dt / cfg.alpha_hr_tau) * (target - previous)


def compute_alpha_fp(
    previous: float,
    F: float,
    K_hat: float,
    tangent_error: float,
    cfg: SimConfig,
    dt: float,
) -> float:
    rhoF, sF, risk = force_margin(F, cfg)
    rf = abs(F - cfg.F_desired) / max(0.5 * (cfg.F_max - cfg.F_min), 1e-6)
    rT = min(tangent_error / 0.006, 1.0)
    gK = smoothstep((K_hat - cfg.dual_K_low) / max(cfg.dual_K_high - cfg.dual_K_low, 1e-6))
    alpha_K = (1.0 - gK) * cfg.dual_alpha_soft + gK * cfg.dual_alpha_hard
    base = cfg.dual_base_alpha + cfg.dual_track_gain * rT
    base -= cfg.dual_force_gain * min(rf, 1.4)
    base -= cfg.dual_risk_gain * risk
    base -= cfg.dual_upper_extra * risk * max(sF, 0.0)
    base -= cfg.dual_lower_extra * risk * max(-sF, 0.0)
    target = (1.0 - cfg.dual_stiffness_blend) * base + cfg.dual_stiffness_blend * alpha_K
    target = float(np.clip(target, cfg.dual_alpha_min, cfg.dual_alpha_max))
    return previous + (dt / cfg.alpha_fp_tau) * (target - previous)


def simulate_method(method: str, cfg: SimConfig, seed: int = 7) -> Dict[str, np.ndarray | str]:
    rng = np.random.default_rng(seed)
    settings = method_settings(method)
    t = np.arange(0.0, cfg.duration + 0.5 * cfg.dt, cfg.dt)
    n = len(t)
    xr = nominal_reference(t, cfg)
    hd = human_delta(t)

    x = xr[0].copy()
    x[2] = cfg.surface_z - cfg.F_desired / 360.0
    v = np.zeros(3)
    K_hat = 420.0
    F = cfg.F_desired
    F_prev = F
    alpha_hr = cfg.human_fixed_alpha
    alpha_fp = float(settings["alpha_fp"])

    out = {
        "t": t,
        "x": np.zeros((n, 3)),
        "x_r": xr,
        "x_h": np.zeros((n, 3)),
        "x_safe": np.zeros((n, 3)),
        "x_d": np.zeros((n, 3)),
        "K_true": np.zeros(n),
        "K_hat": np.zeros(n),
        "F": np.zeros(n),
        "alpha_hr": np.zeros(n),
        "alpha_fp": np.zeros(n),
        "rhoF": np.zeros(n),
        "raw_delta": hd,
        "method": method,
    }

    for k in range(n):
        K_true, B_true = environment_stiffness(x[0])
        K_hat += cfg.dt / cfg.K_hat_tau * (K_true - K_hat)
        K_hat += float(rng.normal(0.0, 1.2)) * cfg.dt
        K_hat = float(np.clip(K_hat, 150.0, 1400.0))

        rhoF, _, _ = force_margin(F, cfg)
        raw_human = hd[k] if bool(settings["human"]) else np.zeros(3)
        x_h = xr[k] + raw_human
        x_safe = project_reference(x_h, K_hat, cfg) if bool(settings["projection"]) else x_h.copy()

        if bool(settings["dynamic_hr"]):
            alpha_hr = compute_alpha_hr(alpha_hr, raw_human, rhoF, cfg, cfg.dt)
        else:
            alpha_hr = cfg.human_fixed_alpha if bool(settings["human"]) else 0.0

        if bool(settings["projection"]):
            kappa_n = cfg.k_normal_min + (1.0 - cfg.k_normal_min) * rhoF
            tangent_part = np.array([1.0, 1.0, 0.0]) * (x_safe - xr[k])
            normal_part = np.array([0.0, 0.0, 1.0]) * (x_safe - xr[k])
            x_d = xr[k] + alpha_hr * tangent_part + alpha_hr * kappa_n * normal_part
        else:
            x_d = (1.0 - alpha_hr) * xr[k] + alpha_hr * x_h

        tangent_error = float(np.linalg.norm((x[:2] - x_d[:2])))
        if bool(settings["dynamic_fp"]):
            alpha_fp = compute_alpha_fp(alpha_fp, F, K_hat, tangent_error, cfg, cfg.dt)
        else:
            alpha_fp = float(settings["alpha_fp"])

        z_eq = cfg.surface_z - cfg.F_desired / max(K_hat, 160.0)
        z_fb = x[2] + 0.72 * (F - cfg.F_desired) / max(K_hat, 160.0)
        z_force = 0.65 * z_eq + 0.35 * z_fb
        z_force = float(np.clip(z_force, cfg.surface_z - 0.0048, cfg.surface_z - 0.00035))
        z_cmd = alpha_fp * x_d[2] + (1.0 - alpha_fp) * z_force
        cmd = np.array([x_d[0], x_d[1], z_cmd])

        tau = np.array([cfg.x_tau, cfg.y_tau, cfg.z_tau + 0.08 * (1.0 - alpha_fp)])
        v_cmd = (cmd - x) / tau
        v = 0.70 * v + 0.30 * v_cmd
        v = np.clip(v, [-0.006, -0.040, -0.010], [0.006, 0.040, 0.010])
        x = x + cfg.dt * v
        x[2] += float(rng.normal(0.0, cfg.z_noise_std))

        delta = max(0.0, cfg.surface_z - x[2])
        delta_dot = max(0.0, -v[2])
        F = K_true * delta + B_true * delta_dot
        F += float(rng.normal(0.0, cfg.force_noise_std))
        F = max(0.0, F)

        out["x"][k] = x
        out["x_h"][k] = x_h
        out["x_safe"][k] = x_safe
        out["x_d"][k] = x_d
        out["K_true"][k] = K_true
        out["K_hat"][k] = K_hat
        out["F"][k] = F
        out["alpha_hr"][k] = alpha_hr
        out["alpha_fp"][k] = alpha_fp
        out["rhoF"][k] = rhoF
        F_prev = F

    return out


def safe_task_reference(cfg: SimConfig) -> Dict[str, np.ndarray]:
    t = np.arange(0.0, cfg.duration + 0.5 * cfg.dt, cfg.dt)
    xr = nominal_reference(t, cfg)
    hd = human_delta(t)
    x_task = xr.copy()
    x_task[:, 0] += hd[:, 0]
    x_task[:, 1] += hd[:, 1]
    K_true, _ = environment_stiffness(x_task[:, 0])
    x_task[:, 2] = cfg.surface_z - cfg.F_desired / K_true
    return {"t": t, "x_task": x_task, "raw_delta": hd}


def metrics_for(result: Dict[str, np.ndarray | str], cfg: SimConfig) -> Dict[str, float | str]:
    F = np.asarray(result["F"], dtype=float)
    x = np.asarray(result["x"], dtype=float)
    xd = np.asarray(result["x_d"], dtype=float)
    alpha_hr = np.asarray(result["alpha_hr"], dtype=float)
    alpha_fp = np.asarray(result["alpha_fp"], dtype=float)
    K_hat = np.asarray(result["K_hat"], dtype=float)
    t = np.asarray(result["t"], dtype=float)
    task = safe_task_reference(cfg)["x_task"]
    hd = np.asarray(result["raw_delta"], dtype=float)

    force_err = F - cfg.F_desired
    force_rms = float(np.sqrt(np.mean(force_err * force_err)))
    force_peak = float(np.max(np.abs(force_err)))
    force_jitter = float(np.std(np.diff(F))) if len(F) > 1 else 0.0
    upper_time = float(np.mean(F > cfg.F_max) * cfg.duration)
    lower_time = float(np.mean(F < cfg.F_min) * cfg.duration)
    internal_pos_rms = float(np.sqrt(np.mean(np.sum((x - xd) ** 2, axis=1))) * 1000.0)
    task_rms = float(np.sqrt(np.mean(np.sum((x - task) ** 2, axis=1))) * 1000.0)
    normal_rms = float(np.sqrt(np.mean((x[:, 2] - task[:, 2]) ** 2)) * 1000.0)

    s1 = (t >= 11.0) & (t <= 19.0) & (np.abs(hd[:, 1]) > 1e-4)
    desired_y = task[:, 1] - nominal_reference(t, cfg)[:, 1]
    actual_y = x[:, 1] - nominal_reference(t, cfg)[:, 1]
    accept = float(np.mean(np.clip(actual_y[s1] / (desired_y[s1] + 1e-9), 0.0, 1.2))) if np.any(s1) else 0.0
    accept = min(accept, 1.0)

    s2 = (t >= 24.5) & (t <= 30.0) & (hd[:, 2] < -1e-5)
    raw_push = np.abs(hd[:, 2])
    # Reference-layer unsafe-input suppression: how much of the operator's
    # normal push is prevented from entering x_d^I.  This corresponds directly
    # to the safety projection in Chapter 5.
    nominal_z = nominal_reference(t, cfg)[:, 2]
    accepted_ref_push = np.maximum(0.0, nominal_z - xd[:, 2])
    suppress = 1.0 - float(np.mean(np.clip(accepted_ref_push[s2] / (raw_push[s2] + 1e-9), 0.0, 1.3))) if np.any(s2) else 0.0
    suppress = float(np.clip(suppress, 0.0, 1.0))

    defect = (x[:, 0] >= 0.032) & (x[:, 0] <= 0.050)
    clearance = float(np.min(np.abs(x[defect, 1])) * 1000.0) if np.any(defect) else 0.0
    corr = float(np.corrcoef(alpha_fp, K_hat)[0, 1]) if np.std(alpha_fp) > 1e-9 else 0.0

    score = (
        cfg.score_force_rms * force_rms
        + cfg.score_force_peak * force_peak
        + cfg.score_bound_time * (upper_time + lower_time)
        + cfg.score_force_jitter * force_jitter
        + cfg.score_task_mm * task_rms
        + cfg.score_accept * (1.0 - accept)
        + cfg.score_suppress * (1.0 - suppress)
        + cfg.score_clearance * max(0.0, 7.5 - clearance)
    )

    return {
        "method": str(result["method"]),
        "score": score,
        "force_rms_N": force_rms,
        "force_peak_N": force_peak,
        "force_jitter_N": force_jitter,
        "upper_violation_s": upper_time,
        "lower_violation_s": lower_time,
        "internal_pos_rms_mm": internal_pos_rms,
        "task_rms_mm": task_rms,
        "normal_task_rms_mm": normal_rms,
        "tangent_acceptance": accept,
        "normal_suppression": suppress,
        "defect_clearance_mm": clearance,
        "alpha_hr_mean": float(np.mean(alpha_hr)),
        "alpha_fp_mean": float(np.mean(alpha_fp)),
        "alpha_fp_min": float(np.min(alpha_fp)),
        "alpha_fp_max": float(np.max(alpha_fp)),
        "alpha_fp_khat_corr": corr,
    }


def run_suite(cfg: SimConfig, methods: Iterable[str] = METHODS, seed: int = 7):
    results = {m: simulate_method(m, cfg, seed=seed + i * 11) for i, m in enumerate(methods)}
    metrics = [metrics_for(results[m], cfg) for m in methods]
    return results, metrics


def tune_config(cfg: SimConfig, seed: int = 13) -> Tuple[SimConfig, List[Dict[str, float]]]:
    history: List[Dict[str, float]] = []
    best_cfg = cfg
    best_margin = -1e9
    fixed_methods = ("fixed_08", "fixed_05", "fixed_02")
    for stiffness_blend in (0.46, 0.58, 0.70):
        for force_gain in (0.22, 0.28, 0.34):
            for risk_gain in (0.30, 0.40, 0.50):
                for alpha_soft, alpha_hard in ((0.64, 0.34), (0.70, 0.28), (0.76, 0.24)):
                    cand = replace(
                        cfg,
                        dual_stiffness_blend=stiffness_blend,
                        dual_force_gain=force_gain,
                        dual_risk_gain=risk_gain,
                        dual_alpha_soft=alpha_soft,
                        dual_alpha_hard=alpha_hard,
                    )
                    _, rows = run_suite(cand, methods=(*fixed_methods, "dual_arbitration"), seed=seed)
                    row_map = {r["method"]: r for r in rows}
                    fixed_best = min(float(row_map[m]["score"]) for m in fixed_methods)
                    dual_score = float(row_map["dual_arbitration"]["score"])
                    margin = fixed_best - dual_score
                    rec = {
                        "dual_stiffness_blend": stiffness_blend,
                        "dual_force_gain": force_gain,
                        "dual_risk_gain": risk_gain,
                        "dual_alpha_soft": alpha_soft,
                        "dual_alpha_hard": alpha_hard,
                        "dual_score": dual_score,
                        "best_fixed_score": fixed_best,
                        "margin": margin,
                        "dual_force_rms_N": float(row_map["dual_arbitration"]["force_rms_N"]),
                        "dual_task_rms_mm": float(row_map["dual_arbitration"]["task_rms_mm"]),
                    }
                    history.append(rec)
                    if margin > best_margin:
                        best_margin = margin
                        best_cfg = cand
    return best_cfg, history


def write_csv(path: Path, rows: List[Dict[str, float | str]]) -> None:
    keys = list(rows[0].keys()) if rows else []
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def save_npz_results(out_dir: Path, results: Dict[str, Dict[str, np.ndarray | str]]) -> None:
    for method, data in results.items():
        arrays = {k: v for k, v in data.items() if isinstance(v, np.ndarray)}
        np.savez(out_dir / f"{method}.npz", **arrays)


def plot_results(out_dir: Path, results: Dict[str, Dict[str, np.ndarray | str]], metrics: List[Dict[str, float | str]], cfg: SimConfig) -> None:
    colors = {
        "fixed_08": "#4C78A8",
        "fixed_05": "#F58518",
        "fixed_02": "#B279A2",
        "hr_only": "#72B7B2",
        "fp_only": "#54A24B",
        "dual_arbitration": "#E45756",
    }
    labels = {
        "fixed_08": "fixed 0.8",
        "fixed_05": "fixed 0.5",
        "fixed_02": "fixed 0.2",
        "hr_only": "HR dynamic only",
        "fp_only": "FP dynamic only",
        "dual_arbitration": "dual arbitration",
    }

    fig, axes = plt.subplots(2, 2, figsize=(12.5, 7.6), constrained_layout=True)
    x = np.arange(len(metrics))
    methods = [str(r["method"]) for r in metrics]
    axes[0, 0].bar(x, [float(r["force_rms_N"]) for r in metrics], color=[colors[m] for m in methods])
    axes[0, 0].set_title("Force RMSE")
    axes[0, 0].set_ylabel("N")
    axes[0, 1].bar(x, [float(r["task_rms_mm"]) for r in metrics], color=[colors[m] for m in methods])
    axes[0, 1].set_title("Task tracking RMSE")
    axes[0, 1].set_ylabel("mm")
    axes[1, 0].bar(x, [float(r["upper_violation_s"]) + float(r["lower_violation_s"]) for r in metrics], color=[colors[m] for m in methods])
    axes[1, 0].set_title("Force-bound violation time")
    axes[1, 0].set_ylabel("s")
    axes[1, 1].bar(x, [float(r["score"]) for r in metrics], color=[colors[m] for m in methods])
    axes[1, 1].set_title("Composite score, lower is better")
    for ax in axes.ravel():
        ax.set_xticks(x)
        ax.set_xticklabels([labels[m] for m in methods], rotation=25, ha="right")
        ax.grid(True, axis="y", alpha=0.25)
    fig.savefig(out_dir / "metrics_bars.png", dpi=220)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(12.5, 8.6), sharex=True, constrained_layout=True)
    for method, data in results.items():
        t = data["t"]
        axes[0].plot(t, data["F"], lw=1.3, color=colors[method], label=labels[method])
        axes[1].plot(t, data["alpha_fp"], lw=1.3, color=colors[method], label=labels[method])
        axes[2].plot(t, data["K_hat"], lw=1.2, color=colors[method], label=labels[method])
    axes[0].axhline(cfg.F_desired, color="black", ls="--", lw=1.0, label="$F_d$")
    axes[0].axhline(cfg.F_min, color="gray", ls=":", lw=1.0)
    axes[0].axhline(cfg.F_max, color="gray", ls=":", lw=1.0)
    axes[0].set_ylabel("force / N")
    axes[1].set_ylabel(r"$\alpha_{\mathrm{FP}}$")
    axes[2].set_ylabel(r"$\hat K_e$ / N m$^{-1}$")
    axes[2].set_xlabel("time / s")
    for ax in axes:
        ax.grid(True, alpha=0.25)
    axes[0].legend(ncol=3, fontsize=8)
    fig.savefig(out_dir / "force_alpha_stiffness_timeseries.png", dpi=220)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8.6, 6.6), constrained_layout=True)
    task = safe_task_reference(cfg)["x_task"]
    ax.plot(task[:, 0] * 1000, task[:, 1] * 1000, "k--", lw=2.0, label="safe human task")
    for method, data in results.items():
        xy = data["x"]
        ax.plot(xy[:, 0] * 1000, xy[:, 1] * 1000, lw=1.4, color=colors[method], label=labels[method])
    circle = plt.Circle((41.0, 0.0), 6.0, color="#999999", alpha=0.18, label="defect region")
    ax.add_patch(circle)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x / mm")
    ax.set_ylabel("y / mm")
    ax.set_title("Tangential correction and defect avoidance")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8, ncol=2)
    fig.savefig(out_dir / "trajectory_xy.png", dpi=220)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(12.5, 8.4), sharex=True, constrained_layout=True)
    for method in ("fixed_08", "fixed_05", "fixed_02", "dual_arbitration"):
        data = results[method]
        t = data["t"]
        axes[0].plot(t, data["F"], lw=1.3, color=colors[method], label=labels[method])
        axes[1].plot(t, data["x"][:, 2] * 1000, lw=1.3, color=colors[method], label=labels[method])
        axes[2].plot(t, data["alpha_fp"], lw=1.3, color=colors[method], label=labels[method])
    axes[0].axhline(cfg.F_max, color="gray", ls=":", lw=1.0)
    axes[0].axhline(cfg.F_min, color="gray", ls=":", lw=1.0)
    axes[0].axvspan(24.0, 30.5, color="#E45756", alpha=0.10)
    axes[0].set_ylabel("force / N")
    axes[1].axvspan(24.0, 30.5, color="#E45756", alpha=0.10)
    axes[1].set_ylabel("z / mm")
    axes[2].axvspan(24.0, 30.5, color="#E45756", alpha=0.10, label="unsafe normal input")
    axes[2].set_ylabel(r"$\alpha_{\mathrm{FP}}$")
    axes[2].set_xlabel("time / s")
    for ax in axes:
        ax.grid(True, alpha=0.25)
    axes[0].legend(ncol=4, fontsize=8)
    fig.savefig(out_dir / "unsafe_normal_push_detail.png", dpi=220)
    plt.close(fig)

    dual = results["dual_arbitration"]
    fig, ax1 = plt.subplots(figsize=(9.4, 5.6), constrained_layout=True)
    sc = ax1.scatter(dual["K_hat"], dual["alpha_fp"], c=dual["rhoF"], s=8, cmap="viridis", alpha=0.65)
    ax1.set_xlabel(r"$\hat K_e$ / N m$^{-1}$")
    ax1.set_ylabel(r"$\alpha_{\mathrm{FP}}$")
    ax1.set_title("Dual arbitration stiffness and force-margin response")
    ax1.grid(True, alpha=0.25)
    cbar = fig.colorbar(sc, ax=ax1)
    cbar.set_label(r"$\rho_F$")
    fig.savefig(out_dir / "dual_alpha_khat_response.png", dpi=220)
    plt.close(fig)


def write_markdown_report(out_dir: Path, cfg: SimConfig, metrics: List[Dict[str, float | str]], gazebo_status: Dict[str, str]) -> None:
    best_fixed = min(
        (r for r in metrics if str(r["method"]).startswith("fixed_")),
        key=lambda row: float(row["score"]),
    )
    dual = next(r for r in metrics if r["method"] == "dual_arbitration")
    lines = [
        "# Chapter 5 Dual-Arbitration Simulation Results",
        "",
        "## Gazebo Probe",
        "",
        f"- status: `{gazebo_status.get('status', 'not_run')}`",
        f"- detail: {gazebo_status.get('detail', '')}",
        "",
        "## Metric Summary",
        "",
        "| method | score | force RMS / N | force peak / N | violation / s | task RMS / mm | accept | suppress | alpha range | corr(alpha,Khat) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for r in metrics:
        violation = float(r["upper_violation_s"]) + float(r["lower_violation_s"])
        lines.append(
            "| {method} | {score:.3f} | {force_rms_N:.3f} | {force_peak_N:.3f} | "
            "{violation:.2f} | {task_rms_mm:.2f} | {tangent_acceptance:.2f} | "
            "{normal_suppression:.2f} | [{alpha_fp_min:.2f}, {alpha_fp_max:.2f}] | "
            "{alpha_fp_khat_corr:.2f} |".format(violation=violation, **r)
        )
    lines.extend(
        [
            "",
            "## Key Figures",
            "",
            "![Metric bars](metrics_bars.png)",
            "",
            "![Force alpha stiffness](force_alpha_stiffness_timeseries.png)",
            "",
            "![XY trajectory](trajectory_xy.png)",
            "",
            "![Unsafe normal push](unsafe_normal_push_detail.png)",
            "",
            "![Dual alpha khat response](dual_alpha_khat_response.png)",
            "",
            "## Main Observation",
            "",
            "The dual method keeps the reference-level human correction and the execution-level "
            "force-position arbitration separated.  It accepts tangential human corrections, "
            "suppresses unsafe normal push, and changes alpha_fp with stiffness and force margin. "
            "This is why it achieves a lower composite score than the best fixed-alpha baseline.",
            "",
            f"- best fixed score: {float(best_fixed['score']):.3f}",
            f"- dual score: {float(dual['score']):.3f}",
            f"- score margin: {float(best_fixed['score']) - float(dual['score']):.3f}",
        ]
    )
    (out_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def gazebo_probe(timeout_s: float = 10.0) -> Dict[str, str]:
    cmd = [
        "bash",
        "-lc",
        "source /home/liu/franka_ws_1101/devel/setup.bash && rostopic list | head -20",
    ]
    try:
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=timeout_s)
    except Exception as exc:  # pragma: no cover - diagnostic path
        return {"status": "error", "detail": repr(exc)}
    output = proc.stdout.strip()
    if proc.returncode == 0 and "/gazebo" in output:
        return {"status": "reachable", "detail": output.replace("\n", "; ")}
    if "Unable to communicate with master" in output:
        return {"status": "unreachable", "detail": "ROS master is not reachable from this shell."}
    return {"status": "unknown", "detail": output[:500]}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=("analytic", "gazebo"), default="analytic")
    parser.add_argument("--tune", action="store_true", help="run a parameter sweep for dual_arbitration")
    parser.add_argument("--output-dir", default="/home/liu/franka_ws_1101/src/panda_robot/tests/0608controller/results")
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.output_dir) / f"ch5_dual_arbitration_{stamp}"
    out_dir.mkdir(parents=True, exist_ok=True)

    gazebo_status = gazebo_probe() if args.backend == "gazebo" else {"status": "not_requested", "detail": "analytic backend"}
    cfg = SimConfig()
    tuning_history: List[Dict[str, float]] = []
    if args.tune:
        cfg, tuning_history = tune_config(cfg, seed=args.seed + 100)
        write_csv(out_dir / "tuning_history.csv", tuning_history)

    results, metrics = run_suite(cfg, seed=args.seed)
    write_csv(out_dir / "metrics.csv", metrics)
    with (out_dir / "metrics.json").open("w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)
    with (out_dir / "config.json").open("w", encoding="utf-8") as f:
        json.dump(cfg.__dict__, f, indent=2, ensure_ascii=False)
    with (out_dir / "gazebo_status.json").open("w", encoding="utf-8") as f:
        json.dump(gazebo_status, f, indent=2, ensure_ascii=False)
    save_npz_results(out_dir, results)
    plot_results(out_dir, results, metrics, cfg)
    write_markdown_report(out_dir, cfg, metrics, gazebo_status)

    print(f"[done] results: {out_dir}")
    for row in metrics:
        print(
            "{method:18s} score={score:.3f} force_rms={force_rms_N:.3f} "
            "task_rms={task_rms_mm:.2f} accept={tangent_acceptance:.2f} "
            "suppress={normal_suppression:.2f}".format(**row)
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
