"""第5章综合实验工具包。

本包提供第5章所需的实验编排、指标计算、绘图、统计聚合、Falcon 实物接口
和硬件预检工具。包内模块彼此分工如下：

- `method_switches`：定义论文对比方法到控制器参数的映射；
- `run_ch5_gazebo_suite`：批量启动 Gazebo 在线实验；
- `ch5_metrics`：计算第5章统一评价指标；
- `ch5_plotting`：生成论文分析图；
- `aggregate_results`：汇总多次实验并导出 LaTeX 表格；
- `falcon_human_bridge`：把 Falcon 主端位移接入第4章参考层；
- `hardware_preflight`：实物实验前检查 ROS2 话题是否在线。
"""
