"""
ieee_style.py
=============
Shared matplotlib styling helper for IEEE-conference / IEEE-transactions
quality figures. Import this module at the top of every plotting script:

    from ieee_style import apply_ieee_style, IEEE_COLORS, IEEE_LINESTYLES, \
        COL_W, DBL_W, save_fig

Design choices follow IEEE author guidelines:
  * Times-roman serif typography (DejaVu Serif fallback if Times is unavailable).
  * Font size 8–10 pt at FINAL print size; rendered figures are sized to
    ~1.0× the intended print size so no further scaling is required.
  * Clean black axes (linewidth ~0.6 pt), inward ticks, sparse gridlines.
  * 6-color colour-blind-friendly palette + matched line-style cycle so
    figures remain interpretable in grayscale photocopy.
  * Vector PDF for diagrams + 600 dpi PNG for raster outputs.

Two reference widths are exposed:
  COL_W = 3.5"   (single-column IEEEtran figure)
  DBL_W = 7.16"  (double-column figure*; recommended for >=6 sub-panels)
"""

import matplotlib as mpl
import matplotlib.pyplot as plt
import os

# IEEE column dimensions (inches)
COL_W = 3.5
DBL_W = 7.16

# Color-blind-friendly palette derived from Wong/Tol; works in grayscale
IEEE_COLORS = [
    "#0072B2",  # deep blue
    "#D55E00",  # vermilion / orange
    "#009E73",  # bluish green
    "#CC79A7",  # reddish purple
    "#56B4E9",  # sky blue
    "#E69F00",  # amber
    "#000000",  # neutral black for emphasis
]

# Line-style cycle paired with colors for grayscale legibility
IEEE_LINESTYLES = ["-", "--", "-.", ":", (0, (3, 1, 1, 1)), (0, (5, 1)), "-"]


def apply_ieee_style():
    """Apply IEEE rcParams. Call once at script start."""
    mpl.rcParams.update({
        # --- Typography (Times-roman family) ---
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "DejaVu Serif", "Liberation Serif"],
        "mathtext.fontset": "stix",          # closest free Times-equivalent for math
        "font.size": 9,
        "axes.titlesize": 9,
        "axes.labelsize": 9,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "legend.fontsize": 7.5,
        "figure.titlesize": 10,

        # --- Lines & markers ---
        "lines.linewidth": 1.1,
        "lines.markersize": 3.5,
        "patch.linewidth": 0.6,

        # --- Axes ---
        "axes.linewidth": 0.7,
        "axes.edgecolor": "#000000",
        "axes.labelcolor": "#000000",
        "axes.spines.top": True,
        "axes.spines.right": True,

        # --- Ticks (inward, IEEE standard) ---
        "xtick.direction": "in",
        "ytick.direction": "in",
        "xtick.major.size": 3.0,
        "ytick.major.size": 3.0,
        "xtick.minor.size": 1.5,
        "ytick.minor.size": 1.5,
        "xtick.major.width": 0.6,
        "ytick.major.width": 0.6,
        "xtick.color": "#000000",
        "ytick.color": "#000000",

        # --- Grid: sparse and subtle ---
        "axes.grid": True,
        "axes.grid.axis": "both",
        "axes.grid.which": "major",
        "grid.color": "#B0B0B0",
        "grid.linestyle": ":",
        "grid.linewidth": 0.4,
        "grid.alpha": 0.6,

        # --- Legend: tight, no frame border by default ---
        "legend.frameon": True,
        "legend.framealpha": 0.92,
        "legend.edgecolor": "#000000",
        "legend.fancybox": False,
        "legend.borderpad": 0.3,
        "legend.handlelength": 2.0,
        "legend.handletextpad": 0.4,
        "legend.columnspacing": 1.0,

        # --- Figure / Save ---
        "figure.dpi": 150,
        "savefig.dpi": 600,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.03,

        # --- Embed fonts as TrueType (avoids Type-3 fonts in IEEE PDFCheck) ---
        "pdf.fonttype": 42,
        "ps.fonttype": 42,

        "axes.unicode_minus": False,
    })


def save_fig(fig, path, formats=("png",)):
    """Save figure to one or more formats next to `path`. Always 600 dpi."""
    base, _ = os.path.splitext(path)
    for fmt in formats:
        out = f"{base}.{fmt}"
        fig.savefig(out, dpi=600 if fmt == "png" else None)
        print(f"  -> saved: {out}")


def fig_subplots(width_in, height_in, nrows=1, ncols=1, **kwargs):
    """Convenience constructor for IEEE-sized figures."""
    fig, axes = plt.subplots(
        nrows=nrows, ncols=ncols, figsize=(width_in, height_in), **kwargs
    )
    return fig, axes
