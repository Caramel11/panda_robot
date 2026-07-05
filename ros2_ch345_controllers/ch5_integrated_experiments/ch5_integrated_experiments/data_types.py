"""第5章数据结构和通用数组读取工具。

第3/4/5章的实验日志统一保存为 `result.npz`。其中三维向量不是直接保存为
形状为 `(N, 3)` 的数组，而是保存为 `prefix_0`、`prefix_1`、`prefix_2`
三个一维序列。本模块提供小型工具，把这些分量重新拼回矩阵，避免各个
指标和绘图脚本重复写同样的数组拼接逻辑。
"""

from dataclasses import dataclass
from typing import Dict

import numpy as np


@dataclass
class Ch5TrialSummary:
    """单次第5章实验的摘要结构。

    该结构目前主要用于表达数据语义，便于后续扩展。如果之后要把
    `summary.json` 强类型化，可以直接复用这里的字段。
    """

    run_dir: str
    method: str
    task_mode: str
    scenario: str
    trial_id: int
    metrics: Dict[str, float]


def vector3(data, prefix):
    """从 `result.npz` 风格的数据字典中读取三维向量序列。

    Args:
        data: `np.load(result.npz)` 后转换得到的字典。
        prefix: 向量前缀，例如 `tool_ref`、`x_h`、`nominal_ref`。

    Returns:
        形状为 `(N, 3)` 的数组，列分别对应 x/y/z。

    Example:
        `vector3(data, "tool_ref")` 会读取 `tool_ref_0`、
        `tool_ref_1`、`tool_ref_2` 并按列拼接。
    """

    return np.column_stack([np.asarray(data[f"{prefix}_{i}"], dtype=float) for i in range(3)])
