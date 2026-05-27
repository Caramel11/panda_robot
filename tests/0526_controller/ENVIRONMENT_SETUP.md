# 0526 控制器环境配置指南

本文档面向当前 no-RCM 控制器：

```text
/home/liu/franka_ws_1101/src/panda_robot/tests/0526_controller/run_no_rcm.py
```

推荐运行组合：

```bash
--strategy online_priority --controller-mode pareto_iter
```

该控制器不是一个孤立脚本，它依赖 `/home/liu/franka_ws_1101/src` 下多个 ROS
功能包共同提供机器人模型、Gazebo 仿真、Franka/Panda Python API、控制器管理、
MoveIt 配置和 KDL 运动学能力。

## 1. 工作区功能包关系

### `panda_robot`

当前控制器的直接 Python API 依赖。

主要作用：

- 提供 `PandaArm`，用于读取关节状态、发送力矩命令、切换控制器、执行关节回零。
- 提供 `PandaKinematics`，用于 tool/flange 正运动学、速度运动学和 Jacobian。
- 包含当前控制器目录 `tests/0526_controller/`。

当前控制器直接导入：

```python
from panda_robot import PandaArm, PandaKinematics
```

`panda_robot/package.xml` 依赖：

- `rospy`
- `roscpp`
- `std_msgs`
- `franka_interface`
- `franka_core_msgs`

### `franka_ros_interface`

`panda_robot` 的底层接口依赖，提供 Franka ROS Interface 风格的控制 API。

相关子包：

- `franka_interface`：机器人接口、控制器管理、状态读取。
- `franka_core_msgs`：Franka 状态和控制相关消息。
- `franka_ros_controllers`：Franka 控制器相关实现。
- `franka_tools`：辅助工具。
- `franka_moveit`：MoveIt 示例和配置入口。

关键系统依赖：

- `libfranka`
- `franka_ros`
- `franka_msgs`
- `franka_control`
- `franka_hw`
- `controller_manager`
- `hardware_interface`
- `realtime_tools`
- `dynamic_reconfigure`

### `panda_simulator`

Gazebo 仿真环境和仿真控制器来源。

相关子包：

- `panda_gazebo`：启动 Gazebo 世界、加载 Panda URDF、spawn 机器人。
- `panda_sim_controllers`：加载仿真关节控制器，包含 position/velocity/effort 控制器。
- `panda_sim_custom_action_server`：提供仿真 action server，使 `PandaArm` 的关节轨迹/夹爪接口可用。
- `panda_sim_moveit`：仿真场景下的 MoveIt `move_group`。
- `panda_hardware_interface`：仿真硬件接口。

本工作区实际存在的 Gazebo 启动文件是：

```bash
roslaunch panda_gazebo panda_world.launch
```

如果本机环境中 `roslaunch panda_simulator simulation.launch` 可用，那通常来自额外别名、
安装包或旧工作区；仅从当前 `/src` 文件看，推荐使用 `panda_gazebo panda_world.launch`。

### `franka_panda_description`

机器人 URDF/Xacro 模型来源。

`panda_gazebo/launch/panda_world.launch` 会通过：

```text
franka_panda_description/robots/panda_arm_hand.urdf.xacro
```

生成 `robot_description`。当前控制器的 KDL 正运动学/Jacobian 也依赖
ROS 参数服务器中的 `robot_description`。

### `panda_moveit_config`

MoveIt 配置包。

主要提供：

- `planning_context.launch`
- `kinematics.yaml`
- 控制器配置
- SRDF/规划参数

当前 no-RCM 控制器主要通过 `PandaArm` 的安全关节运动和仿真 action server 工作；
如果需要完整 MoveIt 能力，应额外启动：

```bash
roslaunch panda_sim_moveit sim_move_group.launch
```

### MoveIt/KDL 支撑包

工作区中还有以下支撑包：

- `moveit`
- `moveit_msgs`
- `moveit_resources`
- `moveit_visual_tools`
- `rviz_visual_tools`
- `geometric_shapes`
- `srdfdom`
- `orocos_kinematics_dynamics`

它们为 MoveIt、KDL、几何模型、SRDF 解析和可视化提供支撑。当前控制器不直接
import 这些包，但 `panda_robot`、`panda_moveit_config`、仿真和运动学链路会间接用到。

## 2. Python 依赖

当前控制器直接使用：

- `numpy`
- `scipy`
- `matplotlib`
- `rospy`
- `geometry_msgs`
- `panda_robot`

其中：

- `scipy.linalg.solve_continuous_are`
- `scipy.linalg.solve_continuous_lyapunov`
- `scipy.spatial.transform.Rotation`

用于 Pareto/Riccati 求解和姿态误差处理。

`panda_robot/setup.py` 还声明：

- `numpy`
- `numpy-quaternion`

`panda_simulator/requirements.txt` 声明：

- `numpy`
- `numpy-quaternion==2020.5.11.13.33.35`

建议安装：

```bash
python3 -m pip install --user numpy scipy matplotlib numpy-quaternion
```

如果需要严格按 simulator 版本：

```bash
python3 -m pip install --user -r /home/liu/franka_ws_1101/src/panda_simulator/requirements.txt
```

## 3. ROS 系统依赖

假设当前使用 ROS Noetic，可使用：

```bash
sudo apt update
sudo apt install \
  ros-noetic-libfranka \
  ros-noetic-franka-ros \
  ros-noetic-panda-moveit-config \
  ros-noetic-gazebo-ros-control \
  ros-noetic-effort-controllers \
  ros-noetic-joint-state-controller \
  ros-noetic-controller-manager \
  ros-noetic-moveit \
  ros-noetic-moveit-commander \
  ros-noetic-moveit-visual-tools \
  ros-noetic-rospy-message-converter \
  ros-noetic-tf-conversions \
  ros-noetic-kdl-parser \
  ros-noetic-orocos-kdl \
  ros-noetic-franka-gripper \
  python3-catkin-tools \
  python3-rosdep
```

如果不是 Noetic，将 `noetic` 替换为当前 `$ROS_DISTRO`：

```bash
echo $ROS_DISTRO
```

也可以用 `rosdep` 统一补齐：

```bash
cd /home/liu/franka_ws_1101
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

如遇 `python-sip` 等旧依赖冲突，可参考 `panda_simulator` README 中的方式跳过：

```bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys python-sip
```

## 4. 编译工作区

推荐使用 `catkin build`：

```bash
cd /home/liu/franka_ws_1101
catkin build
source devel/setup.bash
```

每个新终端都需要：

```bash
source /home/liu/franka_ws_1101/devel/setup.bash
```

可选：写入 shell 配置：

```bash
echo "source /home/liu/franka_ws_1101/devel/setup.bash" >> ~/.bashrc
```

## 5. 仿真启动顺序

### 终端 1：启动 Gazebo Panda

推荐：

```bash
cd /home/liu/franka_ws_1101
source devel/setup.bash
roslaunch panda_gazebo panda_world.launch
```

如不需要夹爪：

```bash
roslaunch panda_gazebo panda_world.launch load_gripper:=false
```

常用参数：

```bash
roslaunch panda_gazebo panda_world.launch gui:=true paused:=false use_custom_action_servers:=true
```

`panda_world.launch` 默认会：

- 加载 `robot_description`
- 启动 Gazebo 空世界
- spawn Panda 模型
- 启动 `panda_sim_controllers`
- 启动仿真 action server
- 发布 `world -> base -> panda_link0` 静态 TF

### 终端 2：可选启动 MoveIt

当前控制器主要使用力矩控制和仿真 action server。若需要 MoveIt `move_group`：

```bash
cd /home/liu/franka_ws_1101
source devel/setup.bash
roslaunch panda_sim_moveit sim_move_group.launch
```

### 终端 3：运行当前控制器

```bash
cd /home/liu/franka_ws_1101
source devel/setup.bash
python3 src/panda_robot/tests/0526_controller/run_no_rcm.py \
  --strategy online_priority \
  --controller-mode pareto_iter \
  --trials 1 \
  --output-dir src/panda_robot/tests/0526_controller/results
```

只验证在线 alpha，不启用 Pareto 迭代控制器：

```bash
python3 src/panda_robot/tests/0526_controller/run_no_rcm.py \
  --strategy online_priority \
  --controller-mode are \
  --trials 1
```

## 6. 运行前检查

### 检查 ROS 包是否可见

```bash
rospack find panda_robot
rospack find panda_gazebo
rospack find panda_sim_controllers
rospack find panda_sim_custom_action_server
rospack find franka_interface
rospack find franka_core_msgs
```

### 检查 robot_description

启动 Gazebo 后：

```bash
rosparam get /robot_description >/tmp/panda_urdf.xml
test -s /tmp/panda_urdf.xml && echo "robot_description OK"
```

### 检查控制器服务

```bash
rosservice list | grep controller_manager
```

应能看到与 `panda_simulator` 或控制器管理相关的服务。

### 检查关节状态

```bash
rostopic echo -n 1 /joint_states
```

或：

```bash
rostopic echo -n 1 /panda_simulator/custom_franka_state_controller/joint_states
```

### 检查 Python 导入

```bash
cd /home/liu/franka_ws_1101
source devel/setup.bash
python3 - <<'PY'
import numpy
import scipy
import rospy
from panda_robot import PandaArm, PandaKinematics
print("Python/ROS imports OK")
PY
```

## 7. 当前控制器输出目录

控制器默认输出到命令行给定的 `--output-dir`，推荐：

```text
/home/liu/franka_ws_1101/src/panda_robot/tests/0526_controller/results
```

每次运行会生成：

```text
results/no_rcm_YYYYmmdd_HHMMSS/online_priority_alpha_t00.npz
```

增益表会生成：

```text
results/coop_gains_no_rcm.npy
results/coop_gains_no_rcm_pareto_iter.npy
```

这些都是可再生成文件，不建议提交到 Git。

## 8. 常见问题

### `ImportError: No module named panda_robot`

原因：没有 source 当前工作区。

处理：

```bash
cd /home/liu/franka_ws_1101
source devel/setup.bash
```

### `PandaArm: Collision Service Not found`

仿真中常见，当前控制器通常可以继续运行。它表示碰撞行为服务不可用，不一定影响
no-RCM 仿真力矩控制。

### 找不到 `simulation.launch`

当前 `/src/panda_simulator` 中没有 `simulation.launch`。使用：

```bash
roslaunch panda_gazebo panda_world.launch
```

### Gazebo 启动后控制器无法发送力矩

检查：

```bash
rosservice list | grep controller_manager
rostopic list | grep panda_simulator
```

确认 `panda_sim_controllers.launch` 已由 `panda_world.launch` include。

### 控制器卡在接近阶段

检查：

- `scan_z` 是否低于 `approach_z`。
- 虚拟环境是否启用，即未传入 `--no-virtual-env`。
- 日志中 `F=...N` 是否能超过 `contact_force_threshold=0.3N`。

### Pareto 模式接触后离开表面

这是早期失败版本出现过的问题。当前修复版必须按每个 `(alpha, K_e)` 网格点迭代
生成增益，不要退回单一 `alpha0=0.5` 外推 P1/P2 的旧写法。

### 旧增益表导致行为异常

删除旧结果后重新运行：

```bash
rm -rf /home/liu/franka_ws_1101/src/panda_robot/tests/0526_controller/results
```

重新执行控制器命令，它会自动预计算增益表。

## 9. 推荐最小复现流程

```bash
# 终端 1
cd /home/liu/franka_ws_1101
source devel/setup.bash
roslaunch panda_gazebo panda_world.launch
```

```bash
# 终端 2
cd /home/liu/franka_ws_1101
source devel/setup.bash
python3 src/panda_robot/tests/0526_controller/run_no_rcm.py \
  --strategy online_priority \
  --controller-mode pareto_iter \
  --trials 1 \
  --output-dir src/panda_robot/tests/0526_controller/results
```

成功时日志应出现：

```text
Contact settled ...
Phase 2: Scanning...
Done. 4000 samples.
Results -> .../results/no_rcm_YYYYmmdd_HHMMSS
```
