"""第4章 ROS2 Python 包安装配置。

这里声明包名、需要安装到 share 目录的配置/launch 文件，以及所有 ``ros2 run``
可调用的 console script 入口。
"""

from glob import glob
from setuptools import find_packages, setup

package_name = "ch4_shared_gt_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        # ROS2 包索引文件，使 ``ros2 pkg`` 和 launch 能找到该包。
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        # 安装 package.xml、参数 YAML 和 launch 文件。
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="liu",
    maintainer_email="liu@example.com",
    description="第4章共享控制仲裁器 Gazebo 实验包，基于 shared_gt_controller 克隆扩展。",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            # 在线 Gazebo 控制器。
            "shared_gt_gazebo_node = ch4_shared_gt_controller.shared_gt_gazebo_node:main",
            # 单次实验绘图。
            "plot_shared_gt_result = ch4_shared_gt_controller.plot_shared_gt_result:main",
            # 多策略/多场景指标对比。
            "compare_ch4_results = ch4_shared_gt_controller.compare_ch4_results:main",
            # 打印手工运行用的 launch 命令。
            "generate_ch4_commands = ch4_shared_gt_controller.generate_ch4_commands:main",
            # 通用批量 Gazebo runner。
            "run_ch4_gazebo_suite = ch4_shared_gt_controller.run_ch4_gazebo_suite:main",
            # 论文推荐对比实验矩阵 runner。
            "run_ch4_comparison_experiments = ch4_shared_gt_controller.run_ch4_comparison_experiments:main",
            # U/S/T/C 圆滑字母轨迹对比实验 runner。
            "run_ch4_smooth_letter_comparison = ch4_shared_gt_controller.run_ch4_smooth_letter_comparison:main",
            # 论文图表/数据资产导出。
            "export_ch4_paper_assets = ch4_shared_gt_controller.export_ch4_paper_assets:main",
            # 从 result.npz 重算论文指标。
            "recompute_ch4_metrics = ch4_shared_gt_controller.ch4_metrics:main",
        ],
    },
)
