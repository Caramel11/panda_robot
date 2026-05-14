import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import seaborn as sns
import os  # 引入 os 模块来处理路径

# 1. 设置绘图风格 (使用默认字体避免报错)
sns.set(style="whitegrid")

# 2. 定义数据文件夹的绝对路径
base_dir = "/home/hjh/ljj_ws/logs/0121"

# 3. 拼接完整路径
files = {
    "master": os.path.join(base_dir, "file_traj_master.xlsx"),
    "slave": os.path.join(base_dir, "file_traj_slave.xlsx"),
    "reshape": os.path.join(base_dir, "file_traj_human.xlsx"),
    "error": os.path.join(base_dir, "error.xlsx"),
    "joints": os.path.join(base_dir, "file_traj_joint.xlsx"),
    "params": os.path.join(base_dir, "arbitrary.xlsx"),
    "dynamics": os.path.join(base_dir, "dFVD.xlsx"),
}

# 4. 读取数据 (自动判断是 Excel 还是 CSV)
data = {}
print(f"正在从目录读取数据: {base_dir} ...")

for key, path in files.items():
    if not os.path.exists(path):
        print(f"❌ 找不到文件: {path}")
        continue

    try:
        # 尝试作为 Excel 读取
        data[key] = pd.read_excel(path)
        print(f"✅ 成功读取 {key}: {data[key].shape}")
    except Exception as e_excel:
        try:
            # 如果 Excel 读取失败，尝试作为 CSV 读取 (以防是命名为 .xlsx 的 csv 文件)
            data[key] = pd.read_csv(path)
            print(f"✅ 成功读取 {key} (CSV模式): {data[key].shape}")
        except Exception as e_csv:
            print(f"❌ 读取错误 {key}: {e_excel}")

# --- 开始绘图 ---
if not data:
    print("没有成功加载任何数据，请检查路径或文件权限。")
else:
    # 创建画布
    fig = plt.figure(figsize=(20, 15))

    # --- 图 1: 3D 轨迹 ---
    ax1 = fig.add_subplot(2, 3, 1, projection="3d")
    if "master" in data and "slave" in data:
        step = 5  # 降采样以加快绘图
        ax1.plot(
            data["master"]["master_x"][::step],
            data["master"]["master_y"][::step],
            data["master"]["master_z"][::step],
            label="Master (Target)",
            color="gray",
            linestyle="--",
            alpha=0.6,
        )
        ax1.plot(
            data["slave"]["slave_x"][::step],
            data["slave"]["slave_y"][::step],
            data["slave"]["slave_z"][::step],
            label="Slave (Actual)",
            color="blue",
            alpha=0.8,
        )
        if "reshape" in data:
            ax1.plot(
                data["reshape"]["slave_x"][::step],
                data["reshape"]["slave_y"][::step],
                data["reshape"]["slave_z"][::step],
                label="Reshape Algo",
                color="green",
                linewidth=2,
            )
        ax1.set_title("3D Trajectory Tracking")
        ax1.set_xlabel("X (m)")
        ax1.set_ylabel("Y (m)")
        ax1.set_zlabel("Z (m)")
        ax1.legend()

    # --- 图 2: 平面投影 (XY Plane) ---
    ax2 = fig.add_subplot(2, 3, 2)
    if "master" in data and "slave" in data:
        ax2.plot(
            data["master"]["master_x"],
            data["master"]["master_y"],
            "k--",
            alpha=0.5,
            label="Master",
        )
        ax2.plot(
            data["slave"]["slave_x"], data["slave"]["slave_y"], "b-", label="Slave"
        )
        if "reshape" in data:
            ax2.plot(
                data["reshape"]["slave_x"],
                data["reshape"]["slave_y"],
                "g-",
                label="Reshape",
            )
        ax2.set_title("2D Projection (XY Plane)")
        ax2.set_xlabel("X Position")
        ax2.set_ylabel("Y Position")
        ax2.legend()

    # --- 图 3: 误差分析 (RCM & Tracking) ---
    ax3 = fig.add_subplot(2, 3, 3)
    if "error" in data:
        ax3_twin = ax3.twinx()
        steps = range(len(data["error"]))
        # 乘以1000转换为mm
        l1 = ax3.plot(
            steps, data["error"]["error_rcm"] * 1000, "r-", label="RCM Error (mm)"
        )
        l2 = ax3_twin.plot(
            steps,
            data["error"]["error_ee"] * 1000,
            "orange",
            label="Tracking Error (mm)",
        )

        ax3.set_title("Precision Analysis")
        ax3.set_xlabel("Time Steps")
        ax3.set_ylabel("RCM Error (mm)", color="r")
        ax3_twin.set_ylabel("End-Effector Error (mm)", color="orange")

        lns = l1 + l2
        labs = [l.get_label() for l in lns]
        ax3.legend(lns, labs, loc="upper right")

    # --- 图 4: 关节角度 ---
    ax4 = fig.add_subplot(2, 3, 4)
    if "joints" in data:
        for i in range(1, 8):
            col_name = f"joint{i}"
            if col_name in data["joints"].columns:
                ax4.plot(data["joints"][col_name], label=f"J{i}")
        ax4.set_title("Joint Angles (q1-q7)")
        ax4.set_xlabel("Time Steps")
        ax4.set_ylabel("Angle (rad)")

    # --- 图 5: 控制参数 Alpha/Beta ---
    ax5 = fig.add_subplot(2, 3, 5)
    if "params" in data:
        # 注意：这里需要确认列名是否为 alpha/beta，如果不是可能会报错，这里加个判断
        if "alpha" in data["params"].columns:
            ax5.plot(data["params"]["alpha"], label="Alpha", color="purple")
            ax5_twin = ax5.twinx()
            ax5_twin.plot(
                data["params"]["beta"], label="Beta", color="brown", linestyle=":"
            )
            ax5.set_title("Adaptive Parameters")
            ax5.set_ylabel("Alpha")
            ax5_twin.set_ylabel("Beta")
            ax5.legend(loc="upper left")

    # --- 图 6: 动力学参数 ---
    ax6 = fig.add_subplot(2, 3, 6)
    if "dynamics" in data:
        cols = ["dF_h", "dV_h", "dD_r"]
        for col in cols:
            if col in data["dynamics"].columns:
                ax6.plot(data["dynamics"][col], label=col)
        ax6.set_title("Dynamics Interaction")
        ax6.legend()

    plt.tight_layout()
    plt.show()  # 如果是在 Jupyter 中直接显示，如果是脚本可以改为 plt.savefig('result.png')
    print("分析完成！")
