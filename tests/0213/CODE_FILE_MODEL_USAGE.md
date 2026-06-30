# 0213 代码文件建模方法与使用说明

`tests/0213` 对应第三章人机共享控制实验。目录中既有真实/仿真实验入口，也有模糊逻辑、卡尔曼估计、微分博弈/MPC 对照、统计分析和绘图脚本。核心问题是：在人类输入、机器人自主目标、RCM 几何约束和障碍/目标切换同时存在时，如何估计人类意图并分配仲裁权重。

## 共享控制模型

任务空间状态通常写成

$$
x_{k+1}=A_dx_k+B_d u_r+B_d u_h,
$$

其中 $u_r$ 为机器人自主控制，$u_h$ 为人类输入或由力/主端映射得到的修正。仲裁参数 $\lambda$ 或 $\alpha$ 用于融合人侧参考与机器人侧参考；RCM 约束通过工具轴线与 trocar 点之间的距离误差进入代价函数或姿态参考。

## 实验入口文件

| 文件 | 建模方法 | 使用方法 |
|---|---|---|
| `real_GT_KF.py` | 微分博弈控制 + 模糊/KF 意图估计主入口。机器人参考、人类输入、RCM 误差和障碍代价共同进入博弈求解，KF 用于平滑仲裁量。 | 真实或仿真环境启动后运行。日志写入 `log_0213/0213_GT_KF/...`。 |
| `real_GT_KF_with_other_target.py` | GT+KF 的目标切换版本。额外引入其他候选目标和意图切换逻辑，用于验证算法面对非原定目标时的适应性。 | 用于论文“其他目标/目标切换”对照。运行后使用 `data_analysis_OtherTarget.py` 或 `analyze_ch3_for_thesis.py` 汇总。 |
| `real_GT_Sigmoid.py` | 微分博弈控制 + Sigmoid 仲裁对照。用固定形式的 Sigmoid 函数替代模糊/KF 推理。 | 用于说明模糊/KF 仲裁相比单调阈值函数的优势。 |
| `real_MPC_KF.py` | MPC 控制 + KF/模糊仲裁对照。MPC 负责预测窗口内的轨迹和约束处理，KF 负责意图平滑。 | 与 `real_GT_KF.py` 比较控制结构差异。 |
| `real_MPC_Sigmoid.py` | MPC + Sigmoid 对照。 | 用作四方法统计中的基线。 |
| `0926_circle_200hz.py` | 早期 200 Hz 圆轨迹/RCM 实验脚本。包含欧拉角/四元数转换、圆轨迹生成、离散系统更新、RCM 姿态计算和实物控制循环。 | 主要用于回溯早期实验。运行前需要确认内部硬编码路径、初始关节角和真实机械臂状态。 |
| `arbitrary.py` | 第三章综合控制库和旧主入口。包含 `FrankaSharedController`、局部重规划、Pareto 求解、RCM 计算、MoveIt 路径规划和目标意图估计。 | 作为共享控制算法参考实现。若单独运行，应先检查 ROS、MoveIt、日志路径和目标/障碍配置。 |

## 模糊、滤波和控制辅助

| 文件 | 建模方法 | 使用方法 |
|---|---|---|
| `fuzzy_logic.py` | 定义绝对仲裁和增量仲裁两套模糊系统。输入通常包含人力强度、速度、距离或误差变化，输出为 $\lambda$ 或 $\Delta\lambda$。 | 被 GT/MPC 实验入口调用。单独调试时实例化 `FuzzyLogicTool` 并调用 `compute()`。 |
| `kalman_filter.py` | `KalmanFilterFusion` 对模糊输出进行状态估计，`DeltaLambdaUpdater` 将增量仲裁累积为有界仲裁参数。 | 用于抑制人类输入噪声和模糊规则跳变。 |
| `lqr.py` | LQR/二次型控制辅助。 | 作为博弈或 MPC 对照中的线性反馈基线。 |
| `debug_main.py` | 调试入口。 | 用于快速检查模型参数、参考轨迹或控制输出，不作为论文主入口。 |

## 统计分析文件

| 文件 | 建模方法 | 使用方法 |
|---|---|---|
| `analyze_ch3_for_thesis.py` | 当前推荐的第三章统计入口。读取 `log_0213` 下的 Excel 日志，计算 RCM 误差、末端误差、交互力、jerk、障碍距离、异常值和显著性检验。 | `python3 tests/0213/analyze_ch3_for_thesis.py`；输出表格和论文图到 `ch3_thesis_outputs/`。 |
| `data_analysis_4method.py` | 四方法统计：GT_KF、GT_Sigmoid、MPC_KF、MPC_Sigmoid。 | 用于生成四方法箱线图和显著性表。 |
| `data_analysis_GTvsMPC.py` | 聚焦 GT 与 MPC 差异。 | 用于分析博弈控制相对 MPC 的统计优势。 |
| `data_analysis_OtherTarget.py` | 目标切换/其他目标场景统计。 | 用于分析非原目标任务中的成功率和误差。 |
| `data_analysis.py` | 旧版统计脚本。 | 保留用于复现旧图。新论文优先使用 `analyze_ch3_for_thesis.py`。 |
| `Boxplot.py` | 早期箱线图绘制脚本。 | 需要检查输入路径后单独运行。 |

## 绘图文件

| 文件 | 建模方法 | 使用方法 |
|---|---|---|
| `plot_traj_gt.py` | 读取单次 GT 日志，绘制末端轨迹、目标、RCM 误差和交互事件。 | 用于生成代表性轨迹图。 |
| `plot_gt_analysis.py` | 绘制 GT 参数同步图，展示力、速度、仲裁量和事件区间。 | 适合说明人机介入前后参数变化。 |
| `plot_gt_sigmoid.py` | GT+Sigmoid 对照图。 | 对比 Sigmoid 仲裁的响应特性。 |
| `plot_mpc.py` | MPC 对照图。 | 对比预测控制轨迹和约束表现。 |
| `plot.py`、`plot0113.py` | 旧版通用绘图脚本。 | 用于复现历史图，运行前检查文件路径。 |
| `plot_fuzzy.py`、`plot_delta_fuzzy.py` | 绘制模糊隶属函数和输出曲面。 | 用于论文中解释绝对/增量模糊规则。 |
| `plot_fuzzy_rule.py`、`plot_delta_fuzzy_rule.py` | 绘制规则表和规则关系图。 | 用于展示模糊规则库。 |
| `plot_system.py` | 绘制第三章系统框架图。 | `python3 tests/0213/plot_system.py`。 |

## 文档和输出

| 文件或目录 | 说明 |
|---|---|
| `METHOD_AND_USAGE.md` | 第三章共享控制实验方法与使用说明。 |
| `CODE_FILE_MODEL_USAGE.md` | 当前文件，按代码文件解释建模和使用方式。 |
| `log_0213/` | 原始 Excel 日志目录，体积较大，默认不提交。 |
| `ch3_thesis_outputs/` | 统计输出目录，默认不提交；需要复现时由脚本重新生成。 |

## 推荐使用流程

1. 采集或准备 `log_0213` 实验日志。
2. 运行 `analyze_ch3_for_thesis.py` 生成统计表和图。
3. 用 `plot_traj_gt.py` 或 `plot_gt_analysis.py` 生成代表性单次图。
4. 用 `plot_fuzzy.py` 和 `plot_delta_fuzzy.py` 检查模糊规则是否与论文描述一致。
5. 真实实验入口运行前，先确认硬编码日志路径、初始位姿、目标点和障碍物位置。
