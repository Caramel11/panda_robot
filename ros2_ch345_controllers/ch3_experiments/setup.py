"""第三章实验 ROS2/Python 功能包安装脚本。"""

from setuptools import find_packages, setup


package_name = "ch3_experiments"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=["ch3_experiments", "ch3_experiments.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (
            "share/" + package_name,
            [
                "package.xml",
                "README.md",
                "CH3_EXPERIMENT_MANUAL.md",
                "CH3_GAZEBO_EXPERIMENT_REPORT.md",
                "CH3_THESIS_EXPERIMENT_WRITING_GUIDE.md",
                "CH3_RCM_EXPERIMENT_MANUAL.md",
            ],
        ),
        (
            "share/" + package_name + "/config",
            [
                "config/ch3_default.yaml",
                "config/ch3_gazebo.yaml",
                "config/ch3_uncertainty.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="liu",
    maintainer_email="2796708472@qq.com",
    description="Chapter 3 force-position cooperative game experiment tools.",
    license="TODO",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "run_ch3_sim = ch3_experiments.run_ch3_sim:main",
            "run_ch3_controller_baseline_comparison = ch3_experiments.run_ch3_controller_baseline_comparison:main",
            "run_ch3_uncertainty = ch3_experiments.run_ch3_uncertainty:main",
            "run_ch3_gazebo_suite = ch3_experiments.run_ch3_gazebo_suite:main",
            "run_ch3_smooth_letter_comparison = ch3_experiments.run_ch3_smooth_letter_comparison:main",
            "run_ch3_rcm_gazebo_suite = ch3_experiments.run_ch3_rcm_gazebo_suite:main",
            "plot_ch3_results = ch3_experiments.plot_ch3_results:main",
            "generate_ch3_report = ch3_experiments.generate_ch3_report:main",
            "analyze_ch3_rcm_result = ch3_experiments.analyze_ch3_rcm_result:main",
            "analyze_ch3_ustc_realistic_comparison = ch3_experiments.analyze_ch3_ustc_realistic_comparison:main",
        ],
    },
)
