"""第三章协同博弈增益表辅助工具。"""

import numpy as np


def load_or_build_gain_table(control_mode="pareto_iter"):
    """Build the same shaped gain table used by no_rcm when available.

    The pure Python simulator uses simplified controllers for speed, but this
    helper lets Chapter 3 scripts validate the actual no_rcm cooperative-game
    gain grid without changing the stable online controller path.
    """
    try:
        from no_rcm.src.gt_controller import CooperativeGameController
    except Exception as exc:
        raise RuntimeError("no_rcm is required to build the cooperative-game gain table") from exc

    ctrl = CooperativeGameController(control_mode=control_mode)
    ctrl.precompute_gains(
        alpha_grid=np.linspace(0.0, 1.0, 21),
        Ke_grid=[50, 80, 100, 150, 200, 300, 500, 800, 1000, 1500, 2000],
    )
    return ctrl


def interpolate_gain(controller, alpha_fp, k_hat):
    """从 no_rcm 协同博弈控制器中按 alpha 和刚度插值增益。"""

    if not hasattr(controller, "get_gain"):
        raise TypeError("controller does not expose get_gain(alpha, K_e)")
    return controller.get_gain(float(alpha_fp), float(k_hat))
