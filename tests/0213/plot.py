import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from pathlib import Path

# 读取Excel文件
root = "logs/0124/0124_GT_KF"
time_str = "GT_KF_20260125_032534"
save_dir = Path(root) / time_str
data1 = pd.read_excel(save_dir / "file_traj_slave.xlsx")
data2 = pd.read_excel(save_dir / "file_traj_master.xlsx")
data3 = pd.read_excel(save_dir / "error.xlsx")

# data4 = pd.read_excel(save_dir / "file_traj_deform.xlsx")
data5 = pd.read_excel(save_dir / "force.xlsx")
data6 = pd.read_excel(save_dir / "arbitrary.xlsx")

data7 = pd.read_excel(save_dir / "file_traj_reshape.xlsx")
data8 = pd.read_excel(save_dir / "file_traj_direct.xlsx")
data9 = pd.read_excel(save_dir / "file_traj_robot.xlsx")
data10 = pd.read_excel(save_dir / "file_traj_human.xlsx")
# data3 = pd.read_excel(save_dir / "file_traj_h.xlsx")
# data4 = pd.read_excel(save_dir / "file_traj_r.xlsx")
# 提取需要绘制的列
data_fuzzy = pd.read_excel(save_dir / "fuzzy.xlsx")
# data_PSI = pd.read_excel(save_dir / "PSI.xlsx")
data_FVD = pd.read_excel(save_dir / "FVD.xlsx")
data_dFVD = pd.read_excel(save_dir / "dFVD.xlsx")


x1 = data1["slave_x"]
y1 = data1["slave_y"]
z1 = data1["slave_z"]
a1 = data1["slave_a"]
b1 = data1["slave_b"]
c1 = data1["slave_c"]

x2 = data2["master_x"]
y2 = data2["master_y"]
z2 = data2["master_z"]
a2 = data2["master_a"]
b2 = data2["master_b"]
c2 = data2["master_c"]

err_rcm = data3["error_rcm"]
err_ee = data3["error_ee"]

# x4 = data4["slave_x"]
# y4 = data4["slave_y"]
# z4 = data4["slave_z"]
# a4 = data4["slave_a"]
# b4 = data4["slave_b"]
# c4 = data4["slave_c"]

x5 = data5["master_x"]
y5 = data5["master_y"]
z5 = data5["master_z"]

alpha = data6["alpha"]
beta = data6["beta"]

# psi = data_PSI["PSI"]

x7 = data7["slave_x"]
y7 = data7["slave_y"]
z7 = data7["slave_z"]
a7 = data7["slave_a"]
b7 = data7["slave_b"]
c7 = data7["slave_c"]

x8 = data8["slave_x"]
y8 = data8["slave_y"]
z8 = data8["slave_z"]
a8 = data8["slave_a"]
b8 = data8["slave_b"]
c8 = data8["slave_c"]

x9 = data9["slave_x"]
y9 = data9["slave_y"]
z9 = data9["slave_z"]
a9 = data9["slave_a"]
b9 = data9["slave_b"]
c9 = data9["slave_c"]

# x10 = data10["slave_x"]
# y10 = data10["slave_y"]
# z10 = data10["slave_z"]
# a10 = data10["slave_a"]
# b10 = data10["slave_b"]
# c10 = data10["slave_c"]

x_fuzzy = data_fuzzy["lambda"]
y_fuzzy = data_fuzzy["delta_lambda"]

F_h = data_FVD["F_h"]
dF_h = data_dFVD["dF_h"]
T_h = data_FVD["T_h"]
dT_h = data_dFVD["dT_h"]
D_r = data_FVD["D_r"]
dD_r = data_dFVD["dD_r"]

# x3 = data3["h_x"]
# y3 = data3["h_y"]
# z3 = data3["h_z"]
# a3 = data3["h_a"]
# b3 = data3["h_b"]
# c3 = data3["h_c"]

# x4 = data4["r_x"]
# y4 = data4["r_y"]
# z4 = data4["r_z"]
# a4 = data4["r_a"]
# b4 = data4["r_b"]
# c4 = data4["r_c"]

ax1 = plt.subplot(projection="3d")  # 创建一个三维的绘图工程

ax1.set_title("traj")  # 设置本图名称
ax1.plot(
    x1, y1, z1, c="r", linewidth=4, label="real"
)  # 绘制数据点 c: 'r'红色，'y'黄色，等颜色
ax1.plot(
    x2, y2, z2, c="b", linewidth=4, label="ref"
)  # 绘制数据点 c: 'r'红色，'y'黄色，等颜色

ax1.plot(x7, y7, z7, linewidth=4, label="ref_reshape")

ax1.plot(x8, y8, z8, linewidth=4, label="ref_direct")

ax1.plot(x9, y9, z9, linewidth=4, label="ref_robot")

# ax1.plot(x10, y10, z10, linewidth=4, label="ref_human")
# ax1.plot(x4, y4, z4, linewidth=4, label="ref_deform")
# ax1.axis("equal")
ax1.set_xlabel("X")  # 设置x坐标轴
ax1.set_ylabel("Y")  # 设置y坐标轴
ax1.set_zlabel("Z")  # 设置z坐标轴

time = np.zeros([np.size(x1), 1])
for i in range(np.size(x1)):
    time[i] = 1 * i
ax1.legend()
plt.grid(True)
# 显示图表
plt.show()

plt.figure(1)  # 创建一个三维的绘图工程
plt.title("Euler")  # 设置本图名称
plt.plot(time, a1, label="slave_1")  # 绘制数据点 c: 'r'红色，'y'黄色，等颜色
plt.plot(time, b1, label="slave_2")  # 绘制数据点 c: 'r'红色，'y'黄色，等颜色
plt.plot(time, c1, label="slave_3")  # 绘制数据点 c: 'r'红色，'y'黄色，等颜色
plt.plot(time, a2, label="master_1")  # 绘制数据点 c: 'r'红色，'y'黄色，等颜色
plt.plot(time, b2, label="master_2")  # 绘制数据点 c: 'r'红色，'y'黄色，等颜色
plt.plot(time, c2, label="master_3")  # 绘制数据点 c: 'r'红色，'y'黄色，等颜色
# plt.plot(time, a4, label="deform_1")  # 绘制数据点 c: 'r'红色，'y'黄色，等颜色
# plt.plot(time, b4, label="deform_2")  # 绘制数据点 c: 'r'红色，'y'黄色，等颜色
# plt.plot(time, c4, label="deform_3")  # 绘制数据点 c: 'r'红色，'y'黄色，等颜色
plt.xlabel("time")  # 设置x坐标轴
plt.ylabel("euler")  # 设置y坐标轴
plt.grid("both")
plt.legend()
# 显示图表
plt.show()

plt.figure(2)  # 创建一个三维的绘图工程
plt.title("Euler_error")  # 设置本图名称
plt.plot(time, a2 - a1, label="error_1")
plt.plot(time, b2 - b1, label="error_2")
plt.plot(time, c2 - c1, label="error_3")
plt.xlabel("step")  # 设置x坐标轴
plt.ylabel("euler_error")  # 设置y坐标轴
plt.grid("both")
plt.legend()
# 显示图表
plt.show()

plt.figure(3)  # 创建一个三维的绘图工程
plt.title("position_error")  # 设置本图名称
plt.plot(time, x2 - x1, label="error_x")
plt.plot(time, y2 - y1, label="error_y")
plt.plot(time, z2 - z1, label="error_z")
plt.xlabel("step")  # 设置x坐标轴
plt.ylabel("position_error")  # 设置y坐标轴
plt.legend()
plt.grid("both")
# 显示图表
plt.show()

err_rcm = np.array(err_rcm)
err_ee = np.array(err_ee)

plt.figure(4)  # 创建一个三维的绘图工程
plt.title("error")  # 设置本图名称
plt.plot(range(err_rcm.size), err_rcm, label="err_rcm")
plt.plot(range(err_ee.size), err_ee, label="err_ee")
plt.xlabel("step")  # 设置x坐标轴
plt.ylabel("error")  # 设置y坐标轴
plt.legend()
plt.grid("both")
# 显示图表
plt.show()

x5 = np.array(x5)
y5 = np.array(y5)
z5 = np.array(z5)
d5 = np.sqrt(x5**2 + y5**2 + z5**2)
plt.figure(5)  # 创建一个三维的绘图工程
plt.title("force")  # 设置本图名称
plt.plot(range(x5.size), x5, label="force_x")
plt.plot(range(y5.size), y5, label="force_y")
plt.plot(range(z5.size), z5, label="force_z")
plt.plot(range(d5.size), d5, label="force_z")
plt.xlabel("step")  # 设置x坐标轴
plt.ylabel("force")  # 设置y坐标轴
plt.legend()
plt.grid("both")
# 显示图表
plt.show()


plt.figure(6)  # 创建一个三维的绘图工程
plt.title("arbitrayr")  # 设置本图名称
plt.plot(range(alpha.size), alpha, label="alpha")
plt.plot(range(beta.size), beta, label="beta")
plt.xlabel("step")  # 设置x坐标轴
plt.ylabel("arbitrayr")  # 设置y坐标轴
plt.legend()
plt.grid("both")
# 显示图表
plt.show()


# plt.figure(7)  # 创建一个三维的绘图工程
# plt.title("PSI")  # 设置本图名称
# plt.plot(range(psi.size), psi, label="psi")
# plt.xlabel("step")  # 设置x坐标轴
# plt.ylabel("psi")  # 设置y坐标轴
# plt.legend()
# plt.grid("both")
# # 显示图表
# plt.show()

plt.figure(7)  # 创建一个三维的绘图工程
plt.title("fuzzy")  # 设置本图名称
plt.plot(range(x_fuzzy.size), x_fuzzy, label="lambda")
plt.plot(range(y_fuzzy.size), y_fuzzy, label="delta_lambda")
plt.xlabel("step")  # 设置x坐标轴
plt.ylabel("fuzzy")  # 设置y坐标轴
plt.legend()
plt.grid("both")
# 显示图表
plt.show()

fig1, axes1 = plt.subplots(3, 1, figsize=(12, 15))

ax2 = axes1[0]
ax2.plot(range(F_h.size), F_h, label="F_h")
ax2.legend()
ax2.grid(True, alpha=0.3)

ax3 = axes1[1]
ax3.plot(range(T_h.size), T_h, label="T_h")
ax3.legend()
ax3.grid(True, alpha=0.3)

ax4 = axes1[2]
ax4.plot(range(D_r.size), D_r, label="D_r")
ax4.legend()
ax4.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()

fig2, axes2 = plt.subplots(3, 1, figsize=(12, 15))

ax5 = axes2[0]
ax5.plot(range(dF_h.size), dF_h, label="dF_h")
ax5.legend()
ax5.grid(True, alpha=0.3)

ax6 = axes2[1]
ax6.plot(range(dT_h.size), dT_h, label="dT_h")
ax6.legend()
ax6.grid(True, alpha=0.3)

ax7 = axes2[2]
ax7.plot(range(dD_r.size), dD_r, label="dD_r")
ax7.legend()
ax7.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()
