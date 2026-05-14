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
    return {k: np.atleast_1d(v).flatten() if v.ndim > 0 else np.array([v])
            for k, v in data.items()}


def collect_files(input_path):
    """如果是目录, 返回所有 .npz; 单个文件直接返回"""
    p = Path(input_path)
    if p.is_dir():
        return sorted(p.glob('*.npz'))
    elif p.is_file():
        return [p]
    else:
        raise FileNotFoundError(input_path)


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
    ap.add_argument('input', help='.npz 文件或包含 .npz 的目录')
    ap.add_argument('--out-dir', default=None,
                    help='图片输出目录 (默认: 输入文件同目录/figures/)')
    ap.add_argument('--show', action='store_true', help='显示图片 (默认仅保存)')
    args = ap.parse_args()

    files = collect_files(args.input)
    if not files:
        print(f"未找到 .npz 文件: {args.input}")
        sys.exit(1)

    if args.out_dir is None:
        base = Path(args.input).parent if Path(args.input).is_file() else Path(args.input)
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
                if not args.show:
                    plt.close(fig)
            except Exception as e:
                print(f'  ✗ {tag} 失败: {e}')

        if args.show:
            plt.show()

    print(f"\n所有图保存到: {args.out_dir}")


if __name__ == '__main__':
    main()
