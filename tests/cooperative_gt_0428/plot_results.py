#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
实验数据可视化分析
==================

读取 run_with_rcm.py 保存的 .npz 数据文件, 绘制以下图表:
  Figure 1 — 位置跟踪 (3 轴 + 范数)
  Figure 2 — 力跟踪 (期望 vs 实际 + 误差)
  Figure 3 — RCM 约束误差
  Figure 4 — α 调度与环境刚度估计
  Figure 5 — 增益时变曲线 (K_total, K_r2, K_ef, K_sf)
  Figure 6 — 综合面板 (单图概览)

用法:
  python plot_results.py                         # 默认处理 results/ 下最近更新的数据目录
  python plot_results.py results/rcm_20260428_103045/coop_fuzzy_t00.npz
  python plot_results.py results/rcm_20260428_103045/   # 处理目录下所有 .npz
"""
import argparse
import os
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

import matplotlib.font_manager as font_manager

# -------------------------- Ubuntu中文配置（强制加载字体，确保无方块） --------------------------
def setup_chinese_font_ubuntu():
    font_path = "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"
    if not os.path.exists(font_path):
        print(f"❌ 字体文件不存在：{font_path}")
        print("👉 请先执行：sudo apt-get install -y fonts-wqy-zenhei")
        plt.rcParams["font.sans-serif"] = ["DejaVu Sans", "sans-serif"]
        plt.rcParams["axes.unicode_minus"] = False
        return
    try:
        font_prop = font_manager.FontProperties(fname=font_path)
        plt.rcParams["font.sans-serif"] = [font_prop.get_name(), "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False  # 负号正常显示（Δλ含负值）
        print(f"✅ 中文配置成功，使用字体：{font_prop.get_name()}")
    except Exception as e:
        print(f"⚠️  中文配置警告：{e}")
        plt.rcParams["font.sans-serif"] = ["DejaVu Sans", "sans-serif"]
        plt.rcParams["axes.unicode_minus"] = False

setup_chinese_font_ubuntu()
# ================================================================
# 数据加载
# ================================================================
def load_npz(path):
    """加载单个 .npz 文件, 返回 dict"""
    data = dict(np.load(path, allow_pickle=True))
    # numpy 数组转一维
    d = {k: np.atleast_1d(v).flatten() if v.ndim > 0 else np.array([v])
         for k, v in data.items()}
    return add_derived_fields(d)


def _fit_length(value, n, fill=0.0):
    """将标量/短数组扩展到长度 n，便于兼容旧日志字段。"""
    arr = np.asarray(value, dtype=float).flatten()
    if len(arr) == n:
        return arr
    if len(arr) == 0:
        return np.full(n, fill, dtype=float)
    if len(arr) == 1:
        return np.full(n, arr[0], dtype=float)
    out = np.full(n, fill, dtype=float)
    m = min(n, len(arr))
    out[:m] = arr[:m]
    if m < n:
        out[m:] = arr[m - 1]
    return out


def add_derived_fields(d):
    """
    补齐新版绘图需要的字段。

    旧版 run_no_rcm_test 只记录 pos_x/y/z、F_measured/F_desired、
    x_desired、error_track/e_r1_norm 等字段，没有 pos_des、pos_err、
    F_err。这里统一派生，避免绘图脚本和数据版本强耦合。
    """
    if 't' not in d:
        return d

    n = len(d['t'])
    if n == 0:
        return d

    for key in ('pos_x', 'pos_y', 'pos_z'):
        if key in d:
            d[key] = _fit_length(d[key], n)

    if 'F_err' not in d and 'F_measured' in d and 'F_desired' in d:
        d['F_measured'] = _fit_length(d['F_measured'], n)
        d['F_desired'] = _fit_length(d['F_desired'], n)
        d['F_err'] = d['F_measured'] - d['F_desired']

    # 期望位置: 新日志优先；旧日志用 x_desired 补 x，y 默认 0，
    # z 无法从旧日志严格恢复时以当前 z 占位，保持图表可生成。
    if 'pos_des_x' not in d:
        if 'x_desired' in d:
            d['pos_des_x'] = _fit_length(d['x_desired'], n)
        elif 'pos_err_x' in d and 'pos_x' in d:
            d['pos_des_x'] = d['pos_x'] - _fit_length(d['pos_err_x'], n)
        elif 'pos_x' in d:
            d['pos_des_x'] = d['pos_x'].copy()

    if 'pos_des_y' not in d:
        if 'pos_err_y' in d and 'pos_y' in d:
            d['pos_des_y'] = d['pos_y'] - _fit_length(d['pos_err_y'], n)
        else:
            d['pos_des_y'] = np.zeros(n)

    if 'pos_des_z' not in d:
        if 'pos_err_z' in d and 'pos_z' in d:
            d['pos_des_z'] = d['pos_z'] - _fit_length(d['pos_err_z'], n)
        elif 'pos_z' in d:
            d['pos_des_z'] = d['pos_z'].copy()

    for axis in ('x', 'y', 'z'):
        pos_key = f'pos_{axis}'
        des_key = f'pos_des_{axis}'
        err_key = f'pos_err_{axis}'
        if err_key not in d and pos_key in d and des_key in d:
            d[err_key] = _fit_length(d[pos_key], n) - _fit_length(d[des_key], n)

    if 'pos_err_norm' not in d:
        if all(k in d for k in ('pos_err_x', 'pos_err_y', 'pos_err_z')):
            d['pos_err_norm'] = np.sqrt(
                d['pos_err_x'] ** 2 + d['pos_err_y'] ** 2 + d['pos_err_z'] ** 2
            )
        elif 'error_track' in d:
            d['pos_err_norm'] = _fit_length(d['error_track'], n)
        elif 'e_r1_norm' in d:
            d['pos_err_norm'] = _fit_length(d['e_r1_norm'], n)
        elif 'e_r' in d:
            d['pos_err_norm'] = np.abs(_fit_length(d['e_r'], n))
        else:
            d['pos_err_norm'] = np.zeros(n)

    if 'error_rcm' not in d:
        d['error_rcm'] = np.zeros(n)

    return d


def collect_files(input_path):
    """如果是目录, 返回所有 .npz; 单个文件直接返回"""
    p = Path(input_path)
    if p.is_dir():
        return sorted(p.glob('*.npz'))
    elif p.is_file():
        return [p]
    else:
        raise FileNotFoundError(input_path)


def find_latest_result_input(results_root):
    """返回 results_root 下最近更新的 .npz 所在目录。"""
    root = Path(results_root)
    if not root.exists():
        raise FileNotFoundError(results_root)

    npz_files = [p for p in root.rglob('*.npz') if p.is_file()]
    if not npz_files:
        raise FileNotFoundError(f"{results_root} 下没有 .npz 文件")

    latest_file = max(npz_files, key=lambda p: p.stat().st_mtime)
    return latest_file.parent, latest_file


# ================================================================
# 图 1: 位置跟踪
# ================================================================
def plot_position_tracking(d, title=""):
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)

    t = d['t']
    axes_labels = ['x', 'y', 'z']

    for i, axis in enumerate(axes_labels):
        ax = axes[i]
        ax.plot(t, d[f'pos_{axis}'] * 1000, label=f'实际 {axis}',
                color='tab:blue', linewidth=1.5)
        ax.plot(t, d[f'pos_des_{axis}'] * 1000, label=f'期望 {axis}',
                color='tab:red', linewidth=1.2, linestyle='--')
        ax.plot(t, d[f'pos_err_{axis}'] * 1000, label=f'误差 {axis}',
                color='tab:gray', linewidth=0.8, alpha=0.6)
        ax.set_ylabel(f'{axis} (mm)')
        ax.legend(loc='upper right', fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.axhline(0, color='k', linewidth=0.4, alpha=0.5)

    # 第四个子图: 误差范数
    ax = axes[3]
    ax.plot(t, d['pos_err_norm'] * 1000, color='tab:purple', linewidth=1.5)
    ax.set_ylabel('误差范数 (mm)')
    ax.set_xlabel('时间 (s)')
    ax.grid(True, alpha=0.3)

    # 标注稳态平均误差
    if len(t) > 50:
        last_quarter = t > (t[-1] * 0.75)
        mean_err = np.mean(d['pos_err_norm'][last_quarter]) * 1000
        ax.text(0.02, 0.95,
                f'后 25% 平均误差: {mean_err:.2f} mm',
                transform=ax.transAxes, fontsize=10,
                verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))

    fig.suptitle(f'图 1 — 工具位置跟踪 ({title})', fontsize=13)
    fig.tight_layout()
    return fig


# ================================================================
# 图 2: 力跟踪
# ================================================================
def plot_force_tracking(d, title=""):
    fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    t = d['t']

    ax = axes[0]
    ax.plot(t, d['F_measured'], label='实际接触力', color='tab:blue',
            linewidth=1.5)
    ax.plot(t, d['F_desired'], label='期望接触力', color='tab:red',
            linewidth=1.2, linestyle='--')
    ax.set_ylabel('接触力 (N)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color='k', linewidth=0.4, alpha=0.5)

    ax = axes[1]
    ax.plot(t, d['F_err'], color='tab:purple', linewidth=1.2)
    ax.fill_between(t, 0, d['F_err'], alpha=0.2, color='tab:purple')
    ax.set_ylabel('力误差 F_meas − F_des (N)')
    ax.set_xlabel('时间 (s)')
    ax.axhline(0, color='k', linewidth=0.6)
    ax.grid(True, alpha=0.3)

    if len(t) > 50:
        last_quarter = t > (t[-1] * 0.75)
        rms = np.sqrt(np.mean(d['F_err'][last_quarter] ** 2))
        max_err = np.max(np.abs(d['F_err'][last_quarter]))
        ax.text(0.02, 0.95,
                f'后 25% 力误差 RMS: {rms:.3f} N\n最大瞬时误差: {max_err:.3f} N',
                transform=ax.transAxes, fontsize=10,
                verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))

    fig.suptitle(f'图 2 — 接触力跟踪 ({title})', fontsize=13)
    fig.tight_layout()
    return fig


# ================================================================
# 图 3: RCM 约束误差
# ================================================================
def plot_rcm_error(d, title=""):
    fig, ax = plt.subplots(figsize=(11, 4))
    t = d['t']

    ax.plot(t, d['error_rcm'] * 1000, color='tab:orange', linewidth=1.2)
    ax.fill_between(t, 0, d['error_rcm'] * 1000, alpha=0.2, color='tab:orange')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('RCM 误差 (mm)')
    ax.set_title(f'图 3 — RCM 约束误差 ({title})', fontsize=13)
    ax.grid(True, alpha=0.3)

    # 安全阈值线 (典型微创手术 < 1 mm)
    ax.axhline(1.0, color='red', linewidth=0.8, linestyle=':',
               label='临床安全上限 (1 mm)')
    ax.legend(loc='upper right')

    if len(t) > 0:
        max_err = np.max(d['error_rcm']) * 1000
        mean_err = np.mean(d['error_rcm']) * 1000
        ax.text(0.02, 0.95,
                f'最大: {max_err:.3f} mm   平均: {mean_err:.3f} mm',
                transform=ax.transAxes, fontsize=10,
                verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))

    fig.tight_layout()
    return fig


# ================================================================
# 图 4: α + K_hat
# ================================================================
def plot_alpha_and_K(d, title=""):
    fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    t = d['t']

    ax = axes[0]
    ax.plot(t, d['alpha'], color='tab:green', linewidth=1.5)
    ax.fill_between(t, 0, d['alpha'], alpha=0.15, color='tab:green')
    ax.set_ylabel('α (位控权重)')
    ax.set_ylim(-0.05, 1.05)
    ax.grid(True, alpha=0.3)
    ax.axhline(0.5, color='k', linewidth=0.4, alpha=0.5, linestyle=':')

    # 阶段标注
    if 'phase' in d:
        phase_names = {0: 'Free', 1: 'Approach', 2: 'Trans', 3: 'Steady', 4: 'Retreat'}
        phase = d['phase']
        # 找出阶段切换点
        changes = np.where(np.diff(phase) != 0)[0]
        change_t = t[changes] if len(changes) > 0 else []
        for ct in change_t:
            ax.axvline(ct, color='gray', linewidth=0.6, alpha=0.5, linestyle=':')

    ax = axes[1]
    ax.plot(t, d['K_hat'], color='tab:brown', linewidth=1.2)
    ax.set_ylabel('环境刚度估计 K̂_e (N/m)')
    ax.set_xlabel('时间 (s)')
    ax.set_yscale('log')
    ax.grid(True, alpha=0.3, which='both')

    fig.suptitle(f'图 4 — α 调度与环境估计 ({title})', fontsize=13)
    fig.tight_layout()
    return fig


# ================================================================
# 图 5: 增益时变曲线
# ================================================================
def plot_gains(d, title=""):
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
    t = d['t']

    panels = [
        ('K_total', '位置增益 K_total = K_v + K_r1 (N/m)', 'tab:blue'),
        ('K_r2',    '速度增益 K_r2 (Ns/m)',                'tab:orange'),
        ('K_ef',    '力增益 K_ef',                          'tab:red'),
        ('K_sf',    '力积分增益 K_sf',                      'tab:purple'),
    ]

    for ax, (key, label, color) in zip(axes.flat, panels):
        if key in d:
            ax.plot(t, d[key], color=color, linewidth=1.2)
            ax.set_ylabel(label, fontsize=10)
            ax.grid(True, alpha=0.3)
            ax.axhline(0, color='k', linewidth=0.4, alpha=0.5)
            if key == 'K_total':
                ax.axhline(100, color='gray', linewidth=0.6,
                           linestyle=':', label='K_v 基线 (100)')
                ax.legend(loc='best', fontsize=9)

    axes[1, 0].set_xlabel('时间 (s)')
    axes[1, 1].set_xlabel('时间 (s)')

    fig.suptitle(f'图 5 — Nash 增益时变 ({title})', fontsize=13)
    fig.tight_layout()
    return fig


# ================================================================
# 图 6: 综合面板
# ================================================================
def plot_summary(d, title=""):
    fig, axes = plt.subplots(3, 2, figsize=(13, 9))
    t = d['t']

    # 左上: 位置 (z 轴, 关键的法向)
    ax = axes[0, 0]
    ax.plot(t, d['pos_z'] * 1000, label='实际 z', color='tab:blue',
            linewidth=1.5)
    ax.plot(t, d['pos_des_z'] * 1000, label='期望 z', color='tab:red',
            linewidth=1.2, linestyle='--')
    ax.set_ylabel('Z 位置 (mm)')
    ax.set_xlabel('时间 (s)')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_title('法向位置 (Z)')

    # 右上: 接触力
    ax = axes[0, 1]
    ax.plot(t, d['F_measured'], label='实际', color='tab:blue', linewidth=1.5)
    ax.plot(t, d['F_desired'], label='期望', color='tab:red',
            linewidth=1.2, linestyle='--')
    ax.set_ylabel('接触力 (N)')
    ax.set_xlabel('时间 (s)')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_title('接触力跟踪')

    # 中左: 位置误差范数
    ax = axes[1, 0]
    ax.plot(t, d['pos_err_norm'] * 1000, color='tab:purple', linewidth=1.2)
    ax.set_ylabel('位置误差范数 (mm)')
    ax.set_xlabel('时间 (s)')
    ax.grid(True, alpha=0.3)
    ax.set_title('位置跟踪误差')

    # 中右: 力误差
    ax = axes[1, 1]
    ax.plot(t, d['F_err'], color='tab:purple', linewidth=1.2)
    ax.fill_between(t, 0, d['F_err'], alpha=0.2, color='tab:purple')
    ax.set_ylabel('力误差 (N)')
    ax.set_xlabel('时间 (s)')
    ax.axhline(0, color='k', linewidth=0.4)
    ax.grid(True, alpha=0.3)
    ax.set_title('力跟踪误差')

    # 下左: RCM 误差
    ax = axes[2, 0]
    ax.plot(t, d['error_rcm'] * 1000, color='tab:orange', linewidth=1.2)
    ax.fill_between(t, 0, d['error_rcm'] * 1000, alpha=0.2,
                    color='tab:orange')
    ax.set_ylabel('RCM 误差 (mm)')
    ax.set_xlabel('时间 (s)')
    ax.grid(True, alpha=0.3)
    ax.set_title('RCM 约束违反度')

    # 下右: α 与 K_hat
    ax = axes[2, 1]
    ax.plot(t, d['alpha'], color='tab:green', linewidth=1.5, label='α')
    ax.set_ylabel('α', color='tab:green')
    ax.set_xlabel('时间 (s)')
    ax.set_ylim(-0.05, 1.05)
    ax.tick_params(axis='y', labelcolor='tab:green')

    ax2 = ax.twinx()
    ax2.plot(t, d['K_hat'], color='tab:brown', linewidth=1.2, label='K̂_e',
             alpha=0.8)
    ax2.set_ylabel('K̂_e (N/m)', color='tab:brown')
    ax2.set_yscale('log')
    ax2.tick_params(axis='y', labelcolor='tab:brown')

    ax.grid(True, alpha=0.3)
    ax.set_title('α 调度 + 环境估计')

    fig.suptitle(f'图 6 — 实验综合面板 ({title})', fontsize=14)
    fig.tight_layout()
    return fig


# ================================================================
# 文本统计摘要
# ================================================================
def print_summary(d, name):
    """终端打印关键统计指标"""
    t = d['t']
    n = len(t)
    duration = t[-1] - t[0] if n > 1 else 0

    pos_err_norm = d['pos_err_norm']
    F_err = d['F_err']
    rcm_err = d['error_rcm']

    last = t > (t[-1] * 0.75) if n > 1 else slice(None)

    print(f"\n{'=' * 70}")
    print(f"  统计摘要 — {name}")
    print(f"{'=' * 70}")
    print(f"  采样数: {n}, 持续: {duration:.2f}s")
    print(f"\n  位置跟踪:")
    print(f"    全程平均误差范数: {np.mean(pos_err_norm)*1000:6.3f} mm")
    print(f"    全程最大误差范数: {np.max(pos_err_norm)*1000:6.3f} mm")
    print(f"    后 25% 平均误差:  {np.mean(pos_err_norm[last])*1000:6.3f} mm")
    print(f"\n  力跟踪:")
    print(f"    全程力误差 RMS:   {np.sqrt(np.mean(F_err**2)):6.3f} N")
    print(f"    最大瞬时力误差:   {np.max(np.abs(F_err)):6.3f} N")
    print(f"    后 25% 力 RMS:    {np.sqrt(np.mean(F_err[last]**2)):6.3f} N")
    print(f"\n  RCM 约束:")
    print(f"    全程 RCM 误差均值: {np.mean(rcm_err)*1000:6.3f} mm")
    print(f"    最大 RCM 误差:    {np.max(rcm_err)*1000:6.3f} mm")
    print(f"\n  α 调度:")
    print(f"    α 范围: [{np.min(d['alpha']):.2f}, {np.max(d['alpha']):.2f}]")
    print(f"    α 均值: {np.mean(d['alpha']):.2f}")


# ================================================================
# 主入口
# ================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('input', nargs='?', default=None,
                    help='.npz 文件或包含 .npz 的目录；未指定时自动选择最新结果目录')
    ap.add_argument('--results-root', default='results',
                    help='自动选择最新数据时搜索的根目录 (默认: results)')
    ap.add_argument('--out-dir', default=None,
                    help='图片输出目录 (默认: 输入文件同目录/figures/)')
    ap.add_argument('--no-show', action='store_true',
                    help='仅保存图片，不显示图窗')
    args = ap.parse_args()

    input_path = args.input
    latest_file = None
    if input_path is None:
        input_path, latest_file = find_latest_result_input(args.results_root)
        print(f"未指定输入，自动选择最近更新数据目录: {input_path}")
        print(f"最近更新文件: {latest_file}")

    files = collect_files(input_path)
    if not files:
        print(f"未找到 .npz 文件: {input_path}")
        sys.exit(1)

    if args.out_dir is None:
        input_path_obj = Path(input_path)
        base = input_path_obj.parent if input_path_obj.is_file() else input_path_obj
        args.out_dir = str(base / 'figures')
    os.makedirs(args.out_dir, exist_ok=True)

    for fp in files:
        name = fp.stem
        print(f"\n处理: {fp}")
        d = load_npz(fp)
        print_summary(d, name)

        plots = [
            ('1_position', plot_position_tracking),
            ('2_force', plot_force_tracking),
            ('3_rcm', plot_rcm_error),
            ('4_alpha_K', plot_alpha_and_K),
            ('5_gains', plot_gains),
            ('6_summary', plot_summary),
        ]

        for tag, fn in plots:
            try:
                fig = fn(d, title=name)
                out_path = os.path.join(args.out_dir, f'{name}_{tag}.png')
                fig.savefig(out_path, dpi=120, bbox_inches='tight')
                print(f'  ✓ {tag} → {out_path}')
                if args.no_show:
                    plt.close(fig)
            except Exception as e:
                print(f'  ✗ {tag} 失败: {e}')

        if not args.no_show:
            plt.show()

    print(f"\n所有图保存到: {args.out_dir}")


if __name__ == '__main__':
    main()
