"""第五章综合实验 ROS2/Python 功能包安装脚本。"""

from glob import glob
from setuptools import find_packages, setup


package_name = "ch5_integrated_experiments"

# ament_python 安装描述。
# data_files 会把 README/实验手册、配置和 launch 安装到 share 目录；
# entry_points 会把 Python 模块注册为 `ros2 run` 可执行命令。
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}", glob("*.md")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="liu",
    maintainer_email="2796708472@qq.com",
    description="Integrated Chapter 5 experiment orchestration and analysis.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            # Gazebo 批量实验、指标计算和绘图。
            "run_ch5_gazebo_suite = ch5_integrated_experiments.run_ch5_gazebo_suite:main",
            "ch5_metrics = ch5_integrated_experiments.ch5_metrics:main",
            "ch5_plotting = ch5_integrated_experiments.ch5_plotting:main",
            "generate_ch5_commands = ch5_integrated_experiments.generate_ch5_commands:main",
            # 实物实验接口和统计导出。
            "ch5_hardware_preflight = ch5_integrated_experiments.hardware_preflight:main",
            "ch5_falcon_human_bridge = ch5_integrated_experiments.falcon_human_bridge:main",
            "ch5_aggregate_results = ch5_integrated_experiments.aggregate_results:main",
            "run_ch5_ustc_comparison = ch5_integrated_experiments.run_ch5_ustc_comparison:main",
            "run_ch5_smooth_letter_comparison = ch5_integrated_experiments.run_ch5_smooth_letter_comparison:main",
            "run_ch5_tvio_challenge = ch5_integrated_experiments.run_ch5_tvio_challenge:main",
            "run_ch345_experiment_matrix = ch5_integrated_experiments.run_ch345_experiment_matrix:main",
        ],
    },
)
