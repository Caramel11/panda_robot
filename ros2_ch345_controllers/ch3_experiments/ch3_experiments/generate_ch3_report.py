"""第三章报告生成命令入口。

当前版本的第三章报告通常由具体 runner 自动生成；该文件保留统一入口，
使 `ros2 run ch3_experiments generate_ch3_report` 可以转发到绘图/报告逻辑。
"""

from .plot_ch3_results import main


if __name__ == "__main__":
    main()
