"""第三章实验时序数据记录器。

控制器、仿真和 Gazebo adapter 都把每个采样时刻的标量写入该 logger。
最终保存的 NPZ 字段名与论文绘图脚本保持一致，避免后处理脚本依赖 ROS bag。
"""

from pathlib import Path

import numpy as np


class Ch3Logger:
    """轻量级字典列表 logger。"""

    def __init__(self):
        """初始化对象参数和运行状态。"""
        self.rows = {}

    def add(self, **kwargs):
        """追加一个采样时刻的字段。"""

        for key, value in kwargs.items():
            self.rows.setdefault(key, []).append(value)

    def as_arrays(self):
        """把内部列表转换为 NumPy 数组字典。"""

        out = {}
        for key, values in self.rows.items():
            try:
                out[key] = np.asarray(values, dtype=float)
            except (TypeError, ValueError):
                out[key] = np.asarray(values)
        return out

    def save_npz(self, path):
        """保存为 NPZ 文件。"""

        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(path, **self.as_arrays())
        return path
