# 参考文献目录梳理与正文引用落点

本文档记录多边控制、腹腔镜、力反馈、自主性、任意仲裁、博弈控制、共享控制、远心运动和其他扩展主题等参考资料的去重阅读结果。资料中存在较多双语、翻译和合并版本，正文引用时按唯一英文论文处理，避免同一文献重复进入参考文献表。

## 主题归类

- 人机共享控制与意图理解：`Continuous Role Adaptation`、`Dimension-Specific Shared Autonomy`、`Physical Interaction as Communication`、`A human activity-aware shared control solution`。
- 博弈控制与动态仲裁：`Differential game theory for versatile pHRI`、`Human-Robot Role Arbitration`、`Adaptive Impedance Controller`、`Shared Control in pHRI`。
- RCM 与固定通道约束：`State of the art in movement around a remote point`、`A Controller to Impose a RCM`、`Dynamic-based RCM Torque Controller`、`A passive admittance controller to enforce RCM`。
- 虚拟夹具、主动约束与力反馈：`Active constraints/virtual fixtures: a survey`、`Dynamic Active Constraints`、`Virtual Fixture Assistance for Suturing`、`Constrained haptic-guided shared control`、`Haptic-guided shared control for needle grasping`。
- 手术自主与多边协作：`A Framework for Multilateral Manipulation in Surgical Tasks`、`A Confidence-Based Shared Control Strategy for STAR`、`Shared Autonomy of a Flexible Manipulator`。

## 已补入正文的位置

- 第 1 章共享控制综述：补充连续角色适应、维度相关仲裁、物理交互作为通信、活动感知共享控制和 STAR 置信共享控制，用于支撑本文将操作者输入解释为监督意图而非底层命令。
- 第 1 章力--位协同综述：补充虚拟夹具、动态主动约束、PCNL 触觉共享控制和针抓取触觉引导，用于支撑接触安全边界和人保持在环的必要性。
- 第 1 章手术机器人综述：补充多边手术操作框架、RCM 综述、RCM 导纳/扭矩/被动控制，以及受限内腔共享自主和儿童内镜虚拟夹具。
- 第 3 章问题描述：补充 RCM 通用性、RCM 软件控制、人机连续控制权协商和物理交互通信。
- 第 3 章仲裁设计：补充触觉共享控制和活动感知共享控制，说明平滑权重转移的文献来源。

## 数据与写作处理原则

- 第三章实验使用四组共享控制方法的重复试验记录。
- 数据处理默认采用同方法内 1.5 倍 IQR 规则剔除明显未完成、记录截断或传感瞬时异常记录，并保留原始指标、异常标记和剔除清单。
- 正文中避免使用“为符合预期而剔除坏点”这类主观表述，统一写成“避免明显未完成任务、传感器瞬时异常或记录截断对统计结果造成偏置”。
