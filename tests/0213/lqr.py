import numpy as np
import control as ct


def compute_K_gt(alpha):
    """
    根据给定的alpha值计算对应的LQR增益K_gt

    参数:
        alpha: 浮点数，0 <= alpha <= 1

    返回:
        K_gt: LQR增益矩阵
    """
    # 初始化基础矩阵
    m2 = np.diag([10, 10, 10])
    d2 = np.diag([300, 300, 300])
    k2 = np.diag([100, 100, 100])
    R_h = np.diag([0.001, 0.001, 0.001])
    R_r = np.diag([0.002, 0.002, 0.002])

    Q_hh = np.diag([1000, 1000, 1000, 0.01, 0.01, 0.01])
    Q_rr = np.diag([1000, 1000, 1000, 0.01, 0.01, 0.01])
    Q_hr = np.diag([0, 0, 0, 0, 0, 0])
    Q_rh = np.diag([0, 0, 0, 0, 0, 0])

    # 构建状态矩阵Ac（修正了原代码中矩阵乘法的错误：* → @）
    sys_Ac = np.vstack(
        (
            np.hstack((np.zeros((3, 3)), np.identity(3))),
            np.hstack((-np.linalg.inv(m2) @ k2, -np.linalg.inv(m2) @ d2)),
        )
    )

    # 构建输入矩阵B
    sys_B = np.vstack((np.zeros((3, 3)), np.linalg.inv(m2)))

    # 计算加权矩阵
    Q_c = alpha * (Q_hh + Q_hr) + (1 - alpha) * (Q_rh + Q_rr)
    R_c = alpha * R_h + (1 - alpha) * R_r

    # 输出矩阵和直接传输矩阵
    C = np.array([[1, 1, 1, 0, 0, 0]])
    D = np.array([[0, 0, 0]])

    # 构建状态空间模型并计算LQR增益
    sysStateSpace = ct.ss(sys_Ac, sys_B, C, D)
    K_gt, solve_P, E = ct.lqr(sysStateSpace, Q_c, R_c)

    return K_gt


# ====================== 预生成并存储所有K_gt数据 ======================
# 生成alpha值序列（0到1，步长0.001，共1001个值）
alpha_values = np.arange(0.0, 1.001, 0.001)
k_gt_database = {}  # 用字典存储，键为保留3位小数的alpha值

print("开始预生成K_gt数据库（共1001组数据）...")
for idx, alpha in enumerate(alpha_values):
    # 进度提示
    if idx % 100 == 0:
        print(f"进度: {idx}/{len(alpha_values)}")

    # 计算并存储（四舍五入到3位小数避免浮点精度问题）
    alpha_key = round(alpha, 3)
    k_gt_database[alpha_key] = compute_K_gt(alpha)

# 保存数据库到文件（后续使用可直接加载，无需重新计算）
np.save("k_gt_database.npy", k_gt_database)
print("数据库生成完成并保存为 k_gt_database.npy")


# ====================== 定义查询函数 ======================
def get_K_gt(alpha):
    """
    根据给定的alpha值获取对应的K_gt

    参数:
        alpha: 浮点数，0 <= alpha <= 1

    返回:
        K_gt: 对应的LQR增益矩阵

    异常:
        ValueError: alpha超出0-1范围
        KeyError: 无对应alpha的K_gt数据
    """
    # 处理浮点精度，四舍五入到3位小数
    alpha_rounded = round(alpha, 3)

    # 范围检查
    if not (0.0 <= alpha_rounded <= 1.0):
        raise ValueError(f"alpha值必须在0到1之间，当前输入为{alpha}")

    # 加载数据库（如果未在内存中）
    if "k_gt_database" not in locals():
        global k_gt_database
        k_gt_database = np.load("k_gt_database.npy", allow_pickle=True).item()

    # 查询并返回
    if alpha_rounded not in k_gt_database:
        raise KeyError(f"未找到alpha={alpha_rounded}对应的K_gt数据")

    return k_gt_database[alpha_rounded]


# ====================== 测试示例 ======================
if __name__ == "__main__":
    # 示例1：查询alpha=0.5对应的K_gt
    alpha1 = 0.5
    K1 = get_K_gt(alpha1)
    print(f"\nalpha={alpha1} 对应的K_gt:")
    print(K1)

    # 示例2：查询alpha=0.123对应的K_gt
    alpha2 = 0.00
    K2 = get_K_gt(alpha2)
    print(f"\nalpha={alpha2} 对应的K_gt:")
    print(K2)

    # 示例3：处理浮点精度问题（如输入0.5000001）
    alpha3 = 1
    K3 = get_K_gt(alpha3)
    print(f"\nalpha={alpha3} (四舍五入为0.5) 对应的K_gt:")
    print(K3)
