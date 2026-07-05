"""第三章 RCM Gazebo 在线实验适配器。

职责与 `gazebo_no_rcm_adapter.py` 类似，但运行的是带 RCM 约束的实验。
适配器统一处理 ROS2 环境、Gazebo 进程、控制器 active 检查和结果分析调用。
"""

import os
import select
import signal
import subprocess
import time
from pathlib import Path


WORKSPACE = Path("/home/liu/franka_ros2_ws")


def ros_bash(command):
    """生成带 ROS2 和工作区环境 source 的 bash 命令。"""

    return (
        "set -e\n"
        "source /opt/ros/humble/setup.bash\n"
        f"source {WORKSPACE}/install/setup.bash\n"
        f"{command}\n"
    )


def start_gazebo_rcm(gui=True, log_file=None):
    """Start Gazebo with a stable 7-DoF effort backend for the RCM controller."""
    world = (
        WORKSPACE
        / "install/franka_gazebo_bringup/share/franka_gazebo_bringup/worlds/empty_no_gravity.sdf"
    )
    gz_args = f"-r {world}" if gui else f"-r -s {world}"
    cmd = ros_bash(
        "ros2 launch franka_gazebo_bringup gazebo_franka_arm_example_controller.launch.py "
        "robot_type:=fr3 "
        "load_gripper:=false "
        "rviz:=false "
        f"gz_args:=\"{gz_args}\" "
        "controller:=no_rcm_effort_controller"
    )
    stdout = None
    if log_file:
        Path(log_file).parent.mkdir(parents=True, exist_ok=True)
        stdout = Path(log_file).open("w", encoding="utf-8")
    env = os.environ.copy()
    if not gui:
        env.setdefault("GZ_SIM_SYSTEM_PLUGIN_PATH", os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""))
    return subprocess.Popen(
        ["bash", "-lc", cmd],
        cwd=str(WORKSPACE),
        stdout=stdout or subprocess.DEVNULL,
        stderr=subprocess.STDOUT if stdout else subprocess.DEVNULL,
        text=True,
        env=env,
        preexec_fn=os.setsid,
    )


def stop_process_group(proc):
    """停止 Gazebo/launch 进程组。"""

    if proc and proc.poll() is None:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        try:
            proc.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=8)


def wait_for_controller(timeout=60.0, process=None):
    """等待 effort 控制器进入 active 状态，并返回最后一次控制器列表输出。"""

    deadline = time.time() + float(timeout)
    last_out = ""
    while time.time() < deadline:
        if process is not None and process.poll() is not None:
            return False, last_out
        cmd = ros_bash("ros2 control list_controllers || true")
        out = subprocess.run(
            ["bash", "-lc", cmd],
            cwd=str(WORKSPACE),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=10,
            check=False,
        ).stdout
        last_out = out
        if "no_rcm_effort_controller" in out and "active" in out:
            return True, out
        time.sleep(1.0)
    return False, last_out


def run_rcm_strategy(
    strategy,
    output_dir,
    trials=1,
    controller_mode="pareto_iter",
    timeout=360,
    use_force_sensor=False,
):
    """运行一次实验或策略，并返回结果目录、进程状态或指标。"""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    force_arg = "" if use_force_sensor else "--no-force-sensor "
    cmd = ros_bash(
        "ros2 run ch3_controller run_with_rcm "
        f"--strategy {strategy} "
        f"--controller-mode {controller_mode} "
        f"--trials {int(trials)} "
        f"--output-dir {output_dir} "
        f"{force_arg}"
        "--local-rcm-task "
        "--no-auto-plot "
        "--plot-no-show "
        "--ros-args "
        "-p cmd_topic:=/no_rcm_effort_controller/commands "
        "-p state_topic:=/joint_states "
        "-p rsp_node:=/robot_state_publisher "
        "-p gravity_compensation_scale:=0.0 "
        "-p joint_move_max_tau_abs:=14.0 "
        "-p joint_move_max_tau_rate:=70.0 "
        "-p joint_move_max_speed:=0.10"
    )
    log_path = output_dir / f"run_with_rcm_{strategy}.log"
    proc = subprocess.Popen(
        ["bash", "-lc", cmd],
        cwd=str(WORKSPACE),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )
    lines = []
    try:
        with log_path.open("w", encoding="utf-8") as log:
            deadline = time.time() + float(timeout)
            while True:
                line = ""
                if proc.stdout is not None:
                    ready, _, _ = select.select([proc.stdout], [], [], 0.5)
                    if ready:
                        line = proc.stdout.readline()
                if line:
                    lines.append(line)
                    log.write(line)
                    log.flush()
                if proc.poll() is not None:
                    rest = proc.stdout.read() if proc.stdout is not None else ""
                    if rest:
                        lines.append(rest)
                        log.write(rest)
                    break
                if time.time() > deadline:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    try:
                        proc.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                        proc.wait(timeout=5)
                    msg = f"\n[run_rcm_strategy] timeout after {timeout:.1f}s; log={log_path}\n"
                    lines.append(msg)
                    log.write(msg)
                    break
        return subprocess.CompletedProcess(
            args=cmd,
            returncode=int(proc.returncode if proc.returncode is not None else 124),
            stdout="".join(lines),
            stderr=None,
        )
    finally:
        if proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=8)


def analyze_latest_result(output_root):
    """分析已有实验结果并生成指标、图表和报告。"""
    root = Path(output_root)
    candidates = sorted(p for p in root.glob("rcm_*") if p.is_dir())
    if not candidates:
        raise FileNotFoundError(f"no rcm_* result directory under {root}")
    latest = candidates[-1]
    cmd = ros_bash(
        "ros2 run ch3_experiments analyze_ch3_rcm_result "
        f"--input {latest} "
        f"--output-dir {latest / 'ch3_rcm_analysis'}"
    )
    result = subprocess.run(
        ["bash", "-lc", cmd],
        cwd=str(WORKSPACE),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    return latest, result
