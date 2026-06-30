# 0630thesis 通用力-位协同对比控制器实现方案

本文档依据以下材料整理：

- `0630thesis_仿真与实物实验方案.md`
- `0630thesis_仿真与实物实验方案_深化版.md`
- `0630thesis_全文框架与章节修改方案.md`
- `0630thesis_全文框架与章节修改方案_深化版.md`
- `6月17日.docx` 导师修改意见

文档目标是给出第三章控制器对比所需的四类通用基线：混合力/位控制、单步 QP 力-位协调控制、线性 MPC 力-位协调控制、固定笛卡尔阻抗控制。每个方案均从建模、控制器设计、伪代码和代码接入方式展开。所有方案都采用最通用、最便于解释的建模方式，不使用现有仓库中为复现某篇论文而写的旧 MPC 共享控制器。

为避免 VS Code 中上下标、范数、转置和求和符号显示异常，本文档中的数学公式统一使用 Markdown/LaTeX 写法：行内变量使用 `$x_d$`，独立公式使用 `$$...$$`。伪代码和命令仍使用代码块。若 VS Code 预览中仍未渲染数学公式，应确认 Markdown 预览开启了数学公式支持，或安装常用 Markdown Math/LaTeX 渲染扩展。

## 0. 设计边界

本文中的四类对比控制器只承担第三章和第五章中的“执行层力-位协同控制器”角色。它们不处理人类输入仲裁，不生成安全人侧参考，也不承担第四章的 `alpha_HR` 动态仲裁功能。若实验中需要人机输入，应由第四章参考层先生成统一的安全期望轨迹 `x_d`，再交给本文四类控制器执行。

统一研究对象为人机协同柔性接触操作。无 RCM 表面扫描和有 RCM 长工具接触只是两类实验边界条件，不应写成不同算法。对控制器而言，两类实验的区别只在误差定义和雅可比映射：

- 无 RCM：控制器直接在工具尖端空间计算平动控制量 `u_tool`，再用工具雅可比映射为关节力矩。
- 有 RCM：控制器仍以接触点或工具尖端为目标，但执行时需要先由 RCM 几何关系得到法兰参考，或在法兰空间构造误差，再用法兰雅可比映射为关节力矩。

建议在代码中把四类控制器统一成同一个接口：

```python
class ForcePositionControllerBase:
    def reset(self):
        pass

    def compute(self, obs, ref, contact, dt):
        """
        Parameters
        ----------
        obs:
            x      当前工具尖端或法兰位置, shape=(3,)
            v      当前工具尖端或法兰线速度, shape=(3,)
            q, dq   可选, 用于限幅或日志
        ref:
            x_d    期望位置, shape=(3,)
            v_d    期望速度, shape=(3,)
            a_d    期望加速度, 可选
            F_d    期望法向接触力, scalar
        contact:
            F      当前接触力向量, shape=(3,)
            F_n    当前法向接触力, scalar
            n      接触法向单位向量, shape=(3,)
            K_hat  估计环境刚度, scalar
            B_hat  估计环境阻尼, scalar
            F_min, F_max 力安全上下界
        dt:
            控制周期

        Returns
        -------
        u:
            3D 平动控制量。力矩接口下解释为任务空间力；速度接口下可解释为期望速度。
        debug:
            字典, 保存误差、权重、QP/MPC 状态、求解时间等日志。
        """
        raise NotImplementedError
```

在当前 ROS/Gazebo 代码结构中，`compute_torque_no_rcm(...)` 和 `compute_torque_with_rcm(...)` 已经承担了“平动控制量 + 姿态控制 + Jacobian 转关节力矩”的职责。因此后续实现时，四类基线控制器只需要替换 `ctrl.compute_control(...)` 的平动部分即可。

## 1. 统一柔性接触建模

### 1.1 任务空间变量

令工具接触点在任务空间中的位置和速度为

$$
\mathbf{x}(t)\in\mathbb{R}^{3},\qquad
\mathbf{v}(t)=\dot{\mathbf{x}}(t).
$$

期望轨迹及其一阶、二阶导数为

$$
\mathbf{x}_{d}(t)\in\mathbb{R}^{3},\qquad
\mathbf{v}_{d}(t)=\dot{\mathbf{x}}_{d}(t),\qquad
\mathbf{a}_{d}(t)=\ddot{\mathbf{x}}_{d}(t).
$$

接触法向为单位向量 $\mathbf{n}_{c}$。平面扫描实验中可取世界坐标系 $z$ 轴或材料表面法向；长工具实验中可取工具尖端接触局部法向。定义法向和切向投影矩阵：

$$
\mathbf{P}_{N}=\mathbf{n}_{c}\mathbf{n}_{c}^{T},
\qquad
\mathbf{P}_{T}=\mathbf{I}-\mathbf{n}_{c}\mathbf{n}_{c}^{T}.
$$

位置误差、速度误差和法向力误差定义为

$$
\mathbf{e}_{p}=\mathbf{x}-\mathbf{x}_{d},\qquad
\mathbf{e}_{v}=\mathbf{v}-\mathbf{v}_{d},
$$

$$
F_{n}=\mathbf{n}_{c}^{T}\mathbf{F},\qquad
e_{f}=F_{n}-F_{d}.
$$

本文统一采用 $\mathbf{e}_{p}=\mathbf{x}-\mathbf{x}_{d}$、$\mathbf{e}_{v}=\mathbf{v}-\mathbf{v}_{d}$、$e_{f}=F_{n}-F_{d}$ 的误差方向。若控制律使用负反馈，则通常写成 $\mathbf{u}=-\mathbf{K}\mathbf{e}$。这样与现有合作博弈控制器中 `e_r1 = measured - reference` 的方向一致。

### 1.2 柔性接触模型

局部柔性接触可用 Kelvin-Voigt 模型近似：

$$
F_{n}=K_{e}\delta+B_{e}\dot{\delta}+d_{f}.
$$

其中 $\delta$ 是沿接触法向的压入深度，$K_{e}$ 是环境表观刚度，$B_{e}$ 是环境阻尼，$d_{f}$ 表示未建模形变、摩擦、传感器噪声和建模误差。对平面表面，可近似取

$$
\delta=\max\left(0,\mathbf{n}_{c}^{T}(\mathbf{x}_{s}-\mathbf{x})\right),
\qquad
\dot{\delta}\approx-\mathbf{n}_{c}^{T}\mathbf{v}.
$$

其中 $\mathbf{x}_{s}$ 是表面参考点。在线实现时不一定需要显式计算 $\delta$，只需要使用测得或估计的 $F_{n}$、$\hat{K}_{e}$ 和 $\hat{B}_{e}$。

在小范围接触、固定采样周期 $\Delta t$ 下，可用一阶线性预测法向力：

$$
F_{n,k+1}\approx
F_{n,k}-\hat{K}_{e}\Delta t\,\mathbf{n}_{c}^{T}\mathbf{v}_{cmd}.
$$

若定义 $\mathbf{v}_{cmd}$ 的正方向为工具实际运动速度，且 $\mathbf{n}_{c}$ 指向离开材料的外法向，则向材料内压入时 $\mathbf{n}_{c}^{T}\mathbf{v}_{cmd}<0$，法向力增加。实际代码中必须通过一次慢速下压实验确认符号；若发现压入时预测力下降，只需把上式符号反过来。

### 1.3 力-位协同控制的判定标准

本文把“力-位协同控制”定义为：控制器不是只跟踪几何轨迹，也不是只调节接触力，而是在同一个闭环内同时利用位置误差、速度误差和接触力误差生成控制量，使工具在完成期望运动的同时维持接触力在期望值或安全范围内。

因此，一个控制器能否称为力-位协同控制，至少应满足四个条件：

1. 控制输入同时受位置目标和力目标影响。若控制律只含 `x-x_d` 而完全不含 `F_n-F_d`，它只是位置控制；若只含 `F_n-F_d` 而不关心 `x_d`，它只是力控制。
2. 位置目标和力目标通过柔性接触模型耦合。柔性材料满足 $F_{n}\approx K_{e}\delta+B_{e}\dot{\delta}$，位置压入量变化会直接改变接触力，因此力控制不能脱离位置运动，位置控制也不能忽略力反馈。
3. 控制器必须给出冲突处理机制。当期望位置要求继续压入、而接触力已经接近上界时，控制器应通过低法向刚度、法向力反馈、优化约束或预测代价削弱压入，而不是盲目跟踪位置。
4. 同一控制器可在切向任务和法向接触任务之间分工。典型柔性扫描中，切向方向主要体现位置轨迹和任务覆盖，法向方向主要体现接触力安全；控制器需要同时处理这两类目标。

这一定义也解释了为什么本文选择阻抗、混合力/位、QP 和 MPC 作为基线。阻抗控制通过虚拟动力学把接触力和运动误差联系起来；混合力/位控制通过选择矩阵在不同方向上分别实现位置和力目标；QP 在单个采样周期内把位置项、力项和力边界放入同一个优化问题；MPC 则在有限预测时域内同时优化未来位置误差、未来力误差和约束违反。四类方法都能实现某种形式的力-位协同，但协同机理、可解释性、实时性和对变刚度环境的适应能力不同。

### 1.4 统一安全监督

所有基线控制器都必须共享同一个硬安全层。硬安全层不是算法性能的一部分，而是实验保护机制：

```text
if F_n > F_hard:
    u = unload_along_normal()
    stop_or_retreat()
if ||u|| > u_max:
    u = u * u_max / ||u||
if ||tau||_inf > tau_max:
    tau = clip(tau, -tau_max, tau_max)
```

论文中应区分软安全边界 `[F_min,F_max]` 和硬安全阈值 `F_hard`。软边界用于评价和控制器约束，硬阈值用于急停或卸载，不能把硬急停后的数据当成控制器正常性能。

## 2. 代码组织建议

建议新增一个轻量模块，例如：

```text
src/panda_robot/tests/0630thesis/controller_baselines/
    __init__.py
    common.py
    impedance_controller.py
    hybrid_force_position_controller.py
    qp_controller.py
    mpc_controller.py
```

若后续要直接嵌入现有 `0603` 实验脚本，也可放到：

```text
src/panda_robot/tests/0603/src/baseline_controllers.py
```

推荐先写成独立模块，再在 `run_no_rcm.py` 或 `run_with_rcm.py` 中通过参数选择：

```bash
python run_no_rcm.py --controller baseline_impedance
python run_no_rcm.py --controller baseline_hybrid
python run_no_rcm.py --controller baseline_qp
python run_no_rcm.py --controller baseline_mpc
python run_no_rcm.py --controller gt_dynamic_alpha
```

统一控制循环伪代码如下：

```python
controller = make_controller(args.controller, cfg)
controller.reset()

while not rospy.is_shutdown():
    obs = read_robot_observation()
    contact = read_or_estimate_contact()
    ref = build_reference(t)

    t_solve0 = time.perf_counter()
    u_trans, dbg = controller.compute(obs, ref, contact, dt)
    dbg["solve_time"] = time.perf_counter() - t_solve0

    u_rot = orientation_controller(...)
    tau = jacobian.T @ np.r_[u_trans, u_rot]
    tau = safety_supervisor.clip_tau(tau)
    robot.exec_torque_cmd(tau)

    logger.log(obs, ref, contact, u_trans, tau, dbg)
```

所有控制器都应记录以下 debug 字段，方便后续统计和画图：

```text
controller_name
e_p_norm, e_t_norm, e_n
e_f
F_n, F_d, F_min, F_max
u_norm
active_constraints
solve_status
solve_time
```

QP 和 MPC 还应额外记录：

```text
qp_cost
qp_iterations
constraint_slack
warm_start_used
```

## 3. 固定笛卡尔阻抗控制

### 3.1 方法定位

固定笛卡尔阻抗控制是最基础、最容易理解、最适合实物部署的接触控制基线。它不显式求解力误差最小化问题，而是通过设定位置误差与控制力之间的动态关系，让机器人在接触中表现为一个虚拟弹簧-阻尼-质量系统。

它的论文作用是说明：固定阻抗参数能够在单一环境中取得稳定接触，但面对变刚度柔性材料时，固定刚度和阻尼难以同时保证轨迹精度和接触力安全。

### 3.2 建模

期望任务空间阻抗模型可写为：

$$
\mathbf{M}_{d}(\ddot{\mathbf{x}}-\ddot{\mathbf{x}}_{d})
+\mathbf{D}_{d}(\dot{\mathbf{x}}-\dot{\mathbf{x}}_{d})
+\mathbf{K}_{d}(\mathbf{x}-\mathbf{x}_{d})
=\mathbf{F}_{ext}-\mathbf{F}_{d,vec}.
$$

其中 $\mathbf{F}_{d,vec}=F_{d}\mathbf{n}_{c}$ 是期望法向接触力向量。若底层直接发送任务空间力，可采用简化 PD 形式：

$$
\mathbf{u}=-\mathbf{K}_{d}\mathbf{e}_{p}
-\mathbf{D}_{d}\mathbf{e}_{v}
+\mathbf{F}_{ff}.
$$

其中

$$
\mathbf{F}_{ff}=F_{d}\mathbf{n}_{c}
$$

表示法向力前馈。切向和法向刚度通常不同：

$$
\mathbf{K}_{d}=k_{T}\mathbf{P}_{T}+k_{N}\mathbf{P}_{N},
\qquad
\mathbf{D}_{d}=d_{T}\mathbf{P}_{T}+d_{N}\mathbf{P}_{N}.
$$

一般取 `k_T > k_N`，使切向轨迹跟踪较硬，法向接触较柔顺。

### 3.3 控制器设计

固定阻抗控制器的核心参数为：

| 参数 | 数学位置 | 物理作用 | 对力-位协同的影响 |
|---|---|---|---|
| $k_T$ | $\mathbf{K}_d$ 的切向刚度 | 决定工具沿表面跟踪轨迹的“硬度” | 增大可提高切向位置精度，但过大时会把表面不平整转化为力扰动 |
| $d_T$ | $\mathbf{D}_d$ 的切向阻尼 | 抑制切向速度误差和超调 | 增大可让扫描更平滑，但过大时响应变慢 |
| $k_N$ | $\mathbf{K}_d$ 的法向刚度 | 决定法向压入深度对位置误差的敏感程度 | 较小 $k_N$ 提供柔顺性，较大 $k_N$ 提高法向位置保持但可能增大力峰值 |
| $d_N$ | $\mathbf{D}_d$ 的法向阻尼 | 抑制接触建立和材料刚度切换时的法向振荡 | 合适 $d_N$ 可降低力抖动，过大则造成卸载和跟随迟缓 |
| $F_d$ | $\mathbf{F}_{ff}=F_d\mathbf{n}_c$ | 法向期望接触力前馈 | 帮助建立目标接触力，但仍需阻抗参数决定实际力波动 |

控制律为：

$$
\mathbf{u}_{T}
=-\mathbf{P}_{T}\left(\mathbf{K}_{d}\mathbf{e}_{p}
+\mathbf{D}_{d}\mathbf{e}_{v}\right),
$$

$$
\mathbf{u}_{N}
=-\mathbf{P}_{N}\left(\mathbf{K}_{d}\mathbf{e}_{p}
+\mathbf{D}_{d}\mathbf{e}_{v}\right)
+F_{d}\mathbf{n}_{c},
$$

$$
\mathbf{u}=\mathbf{u}_{T}+\mathbf{u}_{N}.
$$

如果实物中 `F_d n_c` 的方向与力传感器定义相反，应统一在接触力预处理阶段完成符号转换，不建议每个控制器内部单独改符号。

为了让阻抗基线更公平，可加入“力一致参考”而不是直接改控制律。即根据估计刚度生成法向参考：

$$
\mathbf{x}_{d,N}
=\mathbf{x}_{surface}
-\frac{F_d}{\max(\hat{K}_{e},K_{floor})}\mathbf{n}_{c}.
$$

但这应作为同一阻抗控制器的可选配置，论文中需说明“固定阻抗 + 力一致参考”仍然没有显式动态力-位优先级。

### 协同控制机理：阻抗控制为什么能实现力-位协同

阻抗控制实现力-位协同的关键不在于直接求解 `F_n=F_d`，而在于给机器人末端规定一个期望的力-运动关系。柔性接触中，接触力由压入位移和压入速度产生：

$$
F_{n}\approx K_{e}\delta+B_{e}\dot{\delta}.
$$

当机器人用虚拟弹簧-阻尼跟踪 `x_d` 时，法向位置误差 `P_N e_p` 会通过环境刚度转化为接触力变化；控制器中的法向刚度 `k_N` 和阻尼 `d_N` 则决定机器人面对这种力变化时“硬”还是“软”。因此，阻抗控制实际上把位置误差、速度误差和接触力响应放到同一个动态关系中。

从控制律看：

$$
\mathbf{u}=-\mathbf{K}_{d}\mathbf{e}_{p}
-\mathbf{D}_{d}\mathbf{e}_{v}
+F_{d}\mathbf{n}_{c}.
$$

其中 `-K_d e_p-D_d e_v` 保证位置轨迹跟踪，`F_d n_c` 提供期望接触力前馈，`K_d=k_T P_T+k_N P_N` 又允许切向和法向采用不同刚度。切向通常取较大 `k_T`，使工具沿表面按给定轨迹扫描；法向取较小 `k_N` 和合适 `d_N`，使机器人在接触力扰动下表现出柔顺性，避免像纯位置控制一样刚性压入材料。

换言之，阻抗控制的协同方式是“用虚拟机械阻抗间接协调力和位”：

- 切向方向：主要由 `k_T,d_T` 保证位置轨迹和任务覆盖。
- 法向方向：通过较低 `k_N`、较高阻尼和 `F_d` 前馈建立柔顺接触。
- 接触力变化：不是作为硬约束，而是通过环境-阻抗耦合影响实际运动和控制力。

它能够实现力-位协同，但协同强度由固定参数决定。当材料刚度 `K_e` 升高时，同样的法向位置误差会产生更大的 `F_n`，而固定 `k_N,d_N` 不会主动改变策略；这正是它适合作为基线的原因：它有力-位协同能力，但缺少随刚度和力安全裕度变化的动态优先级。

### 3.4 伪代码

```python
class CartesianImpedanceController(ForcePositionControllerBase):
    def __init__(self, k_t, d_t, k_n, d_n, u_max, use_force_ff=True):
        self.k_t = k_t
        self.d_t = d_t
        self.k_n = k_n
        self.d_n = d_n
        self.u_max = u_max
        self.use_force_ff = use_force_ff

    def compute(self, obs, ref, contact, dt):
        x = obs.x
        v = obs.v
        x_d = ref.x_d
        v_d = ref.v_d
        n = normalize(contact.n)
        Pn = np.outer(n, n)
        Pt = np.eye(3) - Pn

        e_p = x - x_d
        e_v = v - v_d

        K = self.k_t * Pt + self.k_n * Pn
        D = self.d_t * Pt + self.d_n * Pn

        u = -(K @ e_p + D @ e_v)
        if self.use_force_ff:
            u += ref.F_d * n

        u = limit_norm(u, self.u_max)
        debug = {
            "controller_name": "cartesian_impedance",
            "e_t_norm": norm(Pt @ e_p),
            "e_n": float(n @ e_p),
            "e_f": contact.F_n - ref.F_d,
            "F_n": contact.F_n,
            "u_norm": norm(u),
        }
        return u, debug
```

### 3.5 参数整定

建议先用无 RCM 仿真整定，再进入实物。典型顺序为：

1. 关闭力前馈，只用较小 `k_N` 找到稳定接触。
2. 增大 `d_N` 直到接触力不出现明显振荡。
3. 增大 `k_T`，使切向轨迹误差达到可接受范围。
4. 打开 `F_d n_c` 前馈，观察静态力误差是否下降。
5. 固定所有参数后，再与其他方法比较。

参数预期：

| 参数 | 过小现象 | 过大现象 |
|---|---|---|
| `k_T` | 切向轨迹漂移 | 切向振荡、关节力矩增大 |
| `d_T` | 轨迹超调 | 运动迟滞 |
| `k_N` | 法向位置漂移、力建立慢 | 高刚度区力峰值增大 |
| `d_N` | 法向力振荡 | 接触响应迟缓 |

### 3.6 论文中可写的优缺点

优点：结构清楚、实时性强、实物部署简单、参数物理意义明确。

不足：力-位折中被固化在 `K_d,D_d` 参数中；在材料刚度变化时，固定参数不能自动改变位置优先级与力优先级；若 `k_N` 较大，高刚度区容易出现力峰值，若 `k_N` 较小，切换或扫描时法向位置误差较大。

## 4. 混合力/位控制

### 4.1 方法定位

混合力/位控制是经典力控基线。它用选择矩阵显式区分“哪些方向跟踪位置、哪些方向调节力”。柔性表面扫描中最自然的设定是：切向做位置控制，法向做力控制。

它的论文作用是说明：显式方向分解可以直接表达力-位协调，但它依赖稳定的接触法向和固定选择矩阵；在曲面、法向估计误差或刚度突变时，切向/法向完全解耦并不总是充分。

### 4.2 建模

选择矩阵定义为：

$$
\mathbf{S}_{f}=\mathbf{P}_{N}
=\mathbf{n}_{c}\mathbf{n}_{c}^{T},
\qquad
\mathbf{S}_{p}=\mathbf{P}_{T}
=\mathbf{I}-\mathbf{n}_{c}\mathbf{n}_{c}^{T}.
$$

切向位置误差：

$$
\mathbf{e}_{T}
=\mathbf{P}_{T}(\mathbf{x}-\mathbf{x}_{d}),
\qquad
\dot{\mathbf{e}}_{T}
=\mathbf{P}_{T}(\mathbf{v}-\mathbf{v}_{d}).
$$

法向力误差：

$$
e_{F}=F_{n}-F_{d}.
$$

为了消除静态力误差，可定义泄漏积分状态：

$$
\dot{\sigma}_{F}=e_{F}-\varepsilon_{F}\sigma_{F}.
$$

其中 $\varepsilon_{F}>0$ 防止长时间积分漂移。$\sigma_F$ 的作用类似力误差积分项，但由于存在泄漏项 $-\varepsilon_F\sigma_F$，它不会像普通积分器一样在长时间接触或传感器零偏下无限累积。

### 4.3 控制器设计

切向位置控制采用 PD：

$$
\mathbf{u}_{T}
=-\mathbf{K}_{T}\mathbf{e}_{T}
-\mathbf{D}_{T}\dot{\mathbf{e}}_{T}.
$$

法向力控制采用 PI 或 PID。为了实物稳定，建议先用 PI 加速度/力形式：

$$
\mathbf{u}_{N}
=-k_{F}e_{F}\mathbf{n}_{c}
-k_{I}\sigma_{F}\mathbf{n}_{c}
-d_{N}(\mathbf{n}_{c}^{T}\mathbf{v})\mathbf{n}_{c}.
$$

合成控制量：

$$
\mathbf{u}=\mathbf{u}_{T}+\mathbf{u}_{N}.
$$

这里 `u_N` 的符号需要通过实验确认。若 `e_F=F_n-F_d>0` 表示接触力过大，控制器应沿卸载方向输出控制量。若 `n_c` 指向离开材料方向，则卸载方向为 `+n_c`，可写成：

$$
\mathbf{u}_{N}
=k_{F}e_{F}\mathbf{n}_{c}
+k_{I}\sigma_{F}\mathbf{n}_{c}
-d_{N}(\mathbf{n}_{c}^{T}\mathbf{v})\mathbf{n}_{c}.
$$

为了避免符号混乱，推荐在代码中引入 `force_sign`：

```python
unload_dir = contact.unload_dir
u_N = k_f * e_f * unload_dir + k_i * sigma_f * unload_dir
```

其中 `unload_dir` 是力过大时应运动或施力的方向。平面实验中它通常等于表面外法向。

### 协同控制机理：混合力/位控制为什么能实现力-位协同

混合力/位控制实现协同的方式最直接：它承认柔性接触任务中的不同方向承担不同控制目标。对表面扫描任务而言，沿表面切向运动决定扫描范围、覆盖率和轨迹精度；垂直接触面的法向运动决定压入深度和接触力。因此该方法用投影矩阵把任务空间分解为：

$$
\text{切向位置子空间: }\mathbf{P}_{T}
=\mathbf{I}-\mathbf{n}_{c}\mathbf{n}_{c}^{T},
\qquad
\text{法向力控制子空间: }\mathbf{P}_{N}
=\mathbf{n}_{c}\mathbf{n}_{c}^{T}.
$$

随后在切向使用位置控制：

$$
\mathbf{u}_{T}
=-\mathbf{K}_{T}\mathbf{P}_{T}(\mathbf{x}-\mathbf{x}_{d})
-\mathbf{D}_{T}\mathbf{P}_{T}(\mathbf{v}-\mathbf{v}_{d}).
$$

在法向使用力反馈：

$$
\mathbf{u}_{N}\propto F_{n}-F_{d}.
$$

最终合成：

$$
\mathbf{u}=\mathbf{u}_{T}+\mathbf{u}_{N}.
$$

这说明混合力/位控制不是在“位置控制”和“力控制”之间二选一，而是在同一控制周期内对不同任务方向同时施加两类控制目标。切向位置项保证工具沿期望路径运动，法向力项根据 `e_F=F_n-F_d` 调整压入或卸载，使接触力接近期望值。二者通过 `u=u_T+u_N` 同时作用于机器人，因此构成力-位协同。

混合力/位控制的协同逻辑可以概括为：

- 位置目标由切向子空间承担：`P_T e_p -> u_T`。
- 力目标由法向子空间承担：`e_F -> u_N`。
- 接触模型提供耦合关系：法向运动改变压入深度，进而改变 `F_n`。
- 选择矩阵提供分工机制：避免法向力误差直接破坏切向轨迹，也避免切向位置控制强行压入材料。

该方法的优势是解释非常清楚，尤其适合平面扫描、打磨、检测等法向明确的任务。它的局限也来自同一点：如果接触法向估计不准、曲面法向变化快，或 RCM 长工具引入姿态-位置耦合，固定的 `P_T/P_N` 分解就可能不再完全成立；此时切向位置控制和法向力控制会发生耦合，表现为切向误差增大、法向力振荡或接触力调节滞后。

### 4.4 导纳式位置接口版本

如果底层不适合直接发送任务空间力，而是更适合发送速度或位置命令，可把法向力控制写成导纳外环：

$$
v_{N,cmd}
=v_{d,N}+k_{A}(F_{d}-F_{n})
-d_{A}\mathbf{n}_{c}^{T}\mathbf{v},
$$

$$
\mathbf{x}_{d,N}^{new}
=\mathbf{x}_{d,N}+\Delta t\,v_{N,cmd}\mathbf{n}_{c}.
$$

切向仍跟踪原始轨迹：

$$
\mathbf{v}_{T,cmd}
=\mathbf{P}_{T}\mathbf{v}_{d}
-\mathbf{K}_{T}\mathbf{P}_{T}(\mathbf{x}-\mathbf{x}_{d}),
$$

$$
\mathbf{v}_{cmd}
=\mathbf{v}_{T,cmd}+v_{N,cmd}\mathbf{n}_{c}.
$$

这种版本更容易在位置控制接口上实现，但本文当前 `run_no_rcm.py` 更接近任务空间力矩控制，因此优先实现力式版本。

### 4.5 伪代码

```python
class HybridForcePositionController(ForcePositionControllerBase):
    def __init__(self, k_t, d_t, k_f, k_i, d_n, eps_i, u_max):
        self.k_t = k_t
        self.d_t = d_t
        self.k_f = k_f
        self.k_i = k_i
        self.d_n = d_n
        self.eps_i = eps_i
        self.u_max = u_max
        self.sigma_f = 0.0

    def reset(self):
        self.sigma_f = 0.0

    def compute(self, obs, ref, contact, dt):
        x = obs.x
        v = obs.v
        n = normalize(contact.n)
        Pn = np.outer(n, n)
        Pt = np.eye(3) - Pn

        e_p = x - ref.x_d
        e_v = v - ref.v_d
        e_t = Pt @ e_p
        ev_t = Pt @ e_v

        e_f = contact.F_n - ref.F_d
        self.sigma_f += dt * (e_f - self.eps_i * self.sigma_f)
        self.sigma_f = clip(self.sigma_f, -self.sigma_limit, self.sigma_limit)

        u_t = - self.k_t * e_t - self.d_t * ev_t

        # unload_dir 表示 e_f>0 时应卸载的方向。
        unload_dir = contact.unload_dir
        v_n = float(n @ v)
        u_n = (
            self.k_f * e_f * unload_dir
            + self.k_i * self.sigma_f * unload_dir
            - self.d_n * v_n * n
        )

        u = limit_norm(u_t + u_n, self.u_max)
        debug = {
            "controller_name": "hybrid_force_position",
            "e_t_norm": norm(e_t),
            "e_f": e_f,
            "sigma_f": self.sigma_f,
            "F_n": contact.F_n,
            "u_t_norm": norm(u_t),
            "u_n_norm": norm(u_n),
            "u_norm": norm(u),
        }
        return u, debug
```

### 4.6 参数整定

建议先只启用切向 PD，让机器人在不接触或轻接触时稳定跟踪；再启用法向 P；最后加入泄漏积分。

| 参数 | 作用 | 推荐调参现象 |
|---|---|---|
| `k_t` | 切向位置刚度 | 增大可降低切向误差，但可能增加耦合振荡 |
| `d_t` | 切向阻尼 | 增大可抑制超调 |
| `k_f` | 法向力误差比例增益 | 增大可加快力误差收敛，但过大会振荡 |
| `k_i` | 法向力误差积分增益 | 减小静态误差，但过大会造成慢振荡 |
| `eps_i` | 积分泄漏 | 增大可防止风up，但会保留静态力误差 |
| `d_n` | 法向速度阻尼 | 抑制压入和卸载过程中的力振荡 |

### 4.7 论文中可写的优缺点

优点：物理意义直接，切向位置和法向力分工清楚，适合平面表面扫描等法向明确任务。

不足：选择矩阵固定，无法表达位置目标和力目标之间连续优先级；法向估计误差会把有效切向运动误判为法向压入；在柔性刚度突变时，单纯法向 PI 容易出现滞后或振荡；有 RCM 长工具任务中，法向力调节还可能与几何误差耦合。

## 5. 单步 QP 力-位协调控制

### 5.1 方法定位

QP 控制器是最简洁的优化型基线。它不做长时域预测，只在当前控制周期内求解一个二次规划，在位置跟踪、力误差、控制输入和力边界之间做单步折中。

它的论文作用是说明：优化方法可以把力-位目标和约束写得很清楚，但单步 QP 对未来刚度变化没有预测能力；与合作博弈闭式反馈相比，QP 需要在线求解器，实物实时性和失败处理必须额外设计。

### 5.2 决策变量选择

为了通用和易解释，建议 QP 的决策变量采用任务空间期望速度 `v_cmd`，而不是关节力矩。原因是：

1. 速度变量和力预测关系更直观；
2. 无 RCM 与有 RCM 都可以先得到任务空间期望速度，再由已有接口或低层控制器执行；
3. 约束形式简单，适合仿真和实物。

若当前实验必须发送任务空间力，也可把 `v_cmd` 解释为一阶导纳后的中间变量，再用 PD 转为任务空间力：

$$
\mathbf{u}=\mathbf{K}_{v}(\mathbf{v}_{cmd}-\mathbf{v}).
$$

### 5.3 单步预测模型

位置单步预测：

$$
\mathbf{x}_{k+1}
=\mathbf{x}_{k}+\Delta t\,\mathbf{v}_{cmd}.
$$

法向力单步预测：

$$
F_{k+1}
=F_{k}-\hat{K}_{e}\Delta t\,\mathbf{n}_{c}^{T}\mathbf{v}_{cmd}.
$$

若存在阻尼项，可加入：

$$
F_{k+1}
=F_{k}
-\hat{K}_{e}\Delta t\,\mathbf{n}_{c}^{T}\mathbf{v}_{cmd}
-\hat{B}_{e}\mathbf{n}_{c}^{T}(\mathbf{v}_{cmd}-\mathbf{v}_{k}).
$$

为了便于实物调试，初版建议只使用刚度项，并把 `K_hat` 限幅到 `[K_min,K_max]`。

### 5.4 QP 目标函数

构造名义位置跟踪速度：

$$
\mathbf{v}_{nom}
=\mathbf{v}_{d}-\mathbf{K}_{p}(\mathbf{x}-\mathbf{x}_{d}).
$$

QP 目标函数：

$$
\begin{aligned}
\min_{\mathbf{v}_{cmd}}\quad
J_{QP}
=&
\left\|\mathbf{P}_{T}(\mathbf{v}_{cmd}-\mathbf{v}_{nom})\right\|_{\mathbf{W}_{T}}^{2}
+w_{N}\left\|\mathbf{n}_{c}^{T}(\mathbf{v}_{cmd}-\mathbf{v}_{nom})\right\|^{2} \\
&+w_{F}(F_{k+1}-F_{d})^{2}
+w_{u}\left\|\mathbf{v}_{cmd}\right\|^{2}
+w_{\Delta u}\left\|\mathbf{v}_{cmd}-\mathbf{v}_{prev}\right\|^{2}.
\end{aligned}
$$

各项含义：

- 第一项：保持切向位置跟踪；
- 第二项：限制法向速度偏离名义轨迹；
- 第三项：使下一步法向力接近期望力；
- 第四项：抑制过大速度；
- 第五项：抑制速度突变，提高实物平滑性。

其中 $\mathbf{W}_{T}$ 决定切向轨迹跟踪的重要性，$w_N$ 决定法向位置参考是否被保留，$w_F$ 决定力误差在优化中的优先级，$w_u$ 用来限制速度幅值，$w_{\Delta u}$ 用来限制相邻周期指令变化。调参时可以把 $w_T/w_F$ 理解成“位置优先”和“力优先”的相对比例：$w_T$ 较大时更贴近轨迹，$w_F$ 较大时更主动调整法向运动以控制接触力。

### 协同控制机理：QP 控制为什么能实现力-位协同

QP 控制实现力-位协同的核心是把“位置应该怎么走”和“力应该保持多少”写进同一个二次优化问题。它不是先做位置控制、再事后检查力是否越界；也不是只做力控制、让轨迹自然漂移；而是在每个采样周期内直接求一个同时兼顾二者的 `v_cmd`。

首先，位置目标通过名义速度进入优化问题：

$$
\mathbf{v}_{nom}
=\mathbf{v}_{d}-\mathbf{K}_{p}(\mathbf{x}-\mathbf{x}_{d}).
$$

如果没有力约束和力误差项，QP 的最优解会接近 `v_nom`，也就是普通位置伺服控制。其次，力目标通过单步力预测进入优化问题：

$$
F_{k+1}
=F_{k}-\hat{K}_{e}\Delta t\,\mathbf{n}_{c}^{T}\mathbf{v}_{cmd}.
$$

该式说明控制器选择的法向速度会改变下一时刻接触力。因此，QP 在优化 `v_cmd` 时能够预先判断：某个位置跟踪动作是否会让 `F_{k+1}` 偏离 `F_d` 或超过 `[F_min,F_max]`。

目标函数中的位置项和力项构成软协同：

$$
\left\|\mathbf{P}_{T}(\mathbf{v}_{cmd}-\mathbf{v}_{nom})\right\|_{\mathbf{W}_{T}}^{2}
+w_{F}(F_{k+1}-F_{d})^{2}.
$$

第一项要求速度不要偏离位置跟踪需求，第二项要求预测接触力接近期望值。当二者不冲突时，最优解会同时满足轨迹跟踪和力调节；当二者冲突时，例如 `v_nom` 要继续向材料内运动但 `F_k` 已接近上界，力项会把最优解从压入方向拉回，形成对位置目标的约束性修正。

约束条件进一步提供硬协同边界：

$$
F_{min}-s_{low}
\leq F_{k+1}
\leq F_{max}+s_{high}.
$$

这使 QP 不仅“偏好”合适的接触力，而且会在优化可行域中限制下一步力预测。松弛变量 `s_low,s_high` 的作用是避免模型误差或突发扰动导致问题不可行，但由于 `w_slack` 很大，控制器仍会优先满足力边界。

因此，QP 的力-位协同机制可以理解为：

- 位置目标给出名义运动方向和速度。
- 接触模型把候选运动转换为下一步力预测。
- 目标函数在位置误差、力误差和控制平滑性之间做加权折中。
- 约束条件禁止或惩罚会导致力越界的候选运动。

与混合力/位控制相比，QP 不需要把切向和法向完全写成两个独立控制器，而是通过优化自动寻找折中速度；与阻抗控制相比，QP 显式包含力误差和力边界。但它只预测一步，所以协同是局部即时的，对未来刚度突变和长时域轨迹变化考虑不足。

### 5.5 QP 约束

软安全力边界：

$$
F_{min}-s_{low}\leq F_{k+1}\leq F_{max}+s_{high},
\qquad
s_{low}\geq0,\quad s_{high}\geq0.
$$

如果希望避免不可行，可加入松弛变量 `s_low,s_high`，并在目标函数中强惩罚：

$$
w_{s}\left(s_{low}^{2}+s_{high}^{2}\right).
$$

速度边界：

$$
\mathbf{v}_{min}\leq \mathbf{v}_{cmd}\leq \mathbf{v}_{max}.
$$

法向速度边界：

$$
v_{N,min}\leq \mathbf{n}_{c}^{T}\mathbf{v}_{cmd}\leq v_{N,max}.
$$

最终 QP 决策变量可写为：

$$
\mathbf{z}
=
\begin{bmatrix}
v_{cmd,x} & v_{cmd,y} & v_{cmd,z} & s_{low} & s_{high}
\end{bmatrix}^{T}.
$$

标准形式：

$$
\begin{aligned}
\min_{\mathbf{z}}\quad
&\frac{1}{2}\mathbf{z}^{T}\mathbf{H}\mathbf{z}
+\mathbf{g}^{T}\mathbf{z}\\
\text{s.t.}\quad
&\mathbf{A}\mathbf{z}\leq\mathbf{b},\\
&\mathbf{l}_{b}\leq\mathbf{z}\leq\mathbf{u}_{b}.
\end{aligned}
$$

### 5.6 控制器输出

若底层接受速度命令：

```text
output = v_cmd
```

若底层接受任务空间力：

$$
\mathbf{u}=\mathbf{K}_{vel}(\mathbf{v}_{cmd}-\mathbf{v}).
$$

其中 `K_vel` 是速度误差到任务空间力的阻尼增益。为了与其他力矩控制器公平比较，建议所有基线最终都经过同一个 `u_max` 和 `tau_max` 限幅。

### 5.7 伪代码

```python
class SingleStepQPController(ForcePositionControllerBase):
    def __init__(self, weights, limits, solver):
        self.w_t = weights.w_t
        self.w_n = weights.w_n
        self.w_f = weights.w_f
        self.w_u = weights.w_u
        self.w_du = weights.w_du
        self.w_slack = weights.w_slack
        self.k_p = weights.k_p
        self.k_vel = weights.k_vel
        self.limits = limits
        self.solver = solver
        self.v_prev = np.zeros(3)

    def reset(self):
        self.v_prev[:] = 0.0

    def compute(self, obs, ref, contact, dt):
        x, v = obs.x, obs.v
        n = normalize(contact.n)
        Pn = np.outer(n, n)
        Pt = np.eye(3) - Pn
        K_hat = clip(contact.K_hat, self.limits.K_min, self.limits.K_max)

        e_p = x - ref.x_d
        v_nom = ref.v_d - self.k_p * e_p
        F0 = contact.F_n

        # Decision z = [vx, vy, vz, s_low, s_high]
        H, g = build_qp_cost(
            Pt=Pt,
            n=n,
            v_nom=v_nom,
            F0=F0,
            Fd=ref.F_d,
            K_hat=K_hat,
            dt=dt,
            v_prev=self.v_prev,
            weights=self.weights,
        )
        A, b, lb, ub = build_qp_constraints(
            n=n,
            F0=F0,
            K_hat=K_hat,
            dt=dt,
            F_min=contact.F_min,
            F_max=contact.F_max,
            limits=self.limits,
        )

        result = self.solver.solve(H, g, A, b, lb, ub)
        if not result.success:
            v_cmd = fallback_velocity(v_nom, contact, self.limits)
            status = "fallback"
        else:
            v_cmd = result.z[:3]
            status = "solved"

        self.v_prev = v_cmd.copy()
        u = self.k_vel * (v_cmd - v)
        u = limit_norm(u, self.limits.u_max)

        F_pred = F0 - K_hat * dt * float(n @ v_cmd)
        debug = {
            "controller_name": "single_step_qp",
            "solve_status": status,
            "F_pred": F_pred,
            "e_f": F0 - ref.F_d,
            "slack_low": float(result.z[3]) if result.success else np.nan,
            "slack_high": float(result.z[4]) if result.success else np.nan,
            "v_cmd_norm": norm(v_cmd),
            "u_norm": norm(u),
        }
        return u, debug
```

### 5.8 求解器建议

推荐优先级：

1. `osqp`：适合在线 QP，支持 warm start；
2. `scipy.optimize.minimize`：依赖少，适合先验证，但实时性较弱；
3. 手写小维度主动集或投影解：变量只有 3-5 维时可行，但调试成本较高。

为避免实验中求解失败，应设置 fallback：

```python
def fallback_velocity(v_nom, contact, limits):
    if contact.F_n > contact.F_max:
        return project_to_unload_velocity(v_nom, contact.n, limits.v_unload)
    return limit_box(v_nom, limits.v_min, limits.v_max)
```

### 5.9 参数整定

| 参数 | 作用 | 过小现象 | 过大现象 |
|---|---|---|---|
| `w_T` | 切向速度跟踪 | 切向轨迹漂移 | 忽略力约束 |
| `w_F` | 法向力跟踪 | 力误差大 | 法向速度剧烈变化 |
| `w_du` | 平滑性 | 输出跳变 | 响应滞后 |
| `w_slack` | 约束违反惩罚 | 容易越界 | 可行域边界附近保守 |
| `K_hat` 限幅 | 预测稳定性 | 预测不足 | 预测过度、动作保守 |

### 5.10 论文中可写的优缺点

优点：目标函数和约束清晰，能显式处理力边界，便于解释“位置项、力项、平滑项”的权重作用。

不足：只看当前一步，不能提前处理未来刚度切换；依赖在线求解器；模型符号和刚度估计错误会直接影响力预测；在实物上需要 fallback 和限幅机制。

## 6. 线性 MPC 力-位协调控制

### 6.1 方法定位

MPC 是 QP 的有限时域扩展。它在未来 `N` 个采样周期内同时优化位置误差、力误差、输入大小和输入平滑性，并只执行第一步控制。为了满足“通用、便于理解、便于实现”的要求，本文建议采用线性任务空间接触模型，不使用仓库中旧的论文复现式 MPC 共享控制器。

它的论文作用是作为较强优化基线：相比单步 QP，MPC 能提前考虑未来参考轨迹和力边界；相比合作博弈闭式反馈，MPC 在线计算量更大，对模型和求解器更敏感。

### 6.2 状态与输入

选择简单的任务空间线性模型。状态为：

$$
\mathbf{s}_{k}
=
\begin{bmatrix}
\mathbf{x}_{k}^{T} &
\mathbf{v}_{k}^{T} &
F_{n,k} &
\sigma_{F,k}
\end{bmatrix}^{T}
\in\mathbb{R}^{8}.
$$

输入为任务空间力：

$$
\mathbf{u}_{k}\in\mathbb{R}^{3}.
$$

采用二阶导纳近似：

$$
\mathbf{x}_{k+1}
=\mathbf{x}_{k}+\Delta t\,\mathbf{v}_{k},
$$

$$
\mathbf{v}_{k+1}
=\mathbf{v}_{k}
+\Delta t\,\mathbf{M}_{d}^{-1}
\left(\mathbf{u}_{k}-\mathbf{D}_{d}\mathbf{v}_{k}\right),
$$

$$
F_{n,k+1}
=F_{n,k}
-\hat{K}_{e}\Delta t\,\mathbf{n}_{c}^{T}\mathbf{v}_{k+1},
$$

$$
\sigma_{F,k+1}
=\left(1-\varepsilon_{F}\Delta t\right)\sigma_{F,k}
+\Delta t\,(F_{n,k}-F_{d}).
$$

其中 `M_d` 和 `D_d` 是期望任务空间惯性和阻尼，不要求等于真实机器人动力学，只作为预测模型。实际执行仍由底层 Franka 控制器和 `J^T` 映射完成。

若希望进一步简化，也可把输入改为速度 `v_cmd`，此时状态只需 `[x,F_n,σ_F]`，预测模型更稳。但为了与阻抗、混合力/位和合作博弈控制器统一输出任务空间力，推荐先使用力输入版本。

### 6.3 离散线性模型

在一个 MPC 求解周期内，固定 `n_c`、`K_hat`、`M_d` 和 `D_d`，得到线性时变或线性定常模型：

$$
\mathbf{s}_{i+1}
=\mathbf{A}_{i}\mathbf{s}_{i}
+\mathbf{B}_{i}\mathbf{u}_{i}
+\mathbf{c}_{i}.
$$

若预测时域内使用常量 `n_c,K_hat,F_d`，可简化为：

$$
\mathbf{s}_{i+1}
=\mathbf{A}\mathbf{s}_{i}
+\mathbf{B}\mathbf{u}_{i}
+\mathbf{c}.
$$

其中 `c` 包含 `F_d` 对 `σ_F` 积分的影响。实现中可不显式写出矩阵，而用循环构造预测等式约束；但论文中建议给出矩阵形式，便于说明该 MPC 是通用线性接触模型。

### 6.4 MPC 目标函数

预测时域长度为 `N`。目标函数：

$$
\begin{aligned}
\min_{\mathbf{u}_{0},\ldots,\mathbf{u}_{N-1}}\quad
J_{MPC}
=&
\sum_{i=1}^{N}
\left\|\mathbf{P}_{T}(\mathbf{x}_{i}-\mathbf{x}_{d,i})\right\|_{\mathbf{Q}_{T}}^{2}
+\sum_{i=1}^{N}
q_{N}\left\|\mathbf{n}_{c}^{T}(\mathbf{x}_{i}-\mathbf{x}_{d,i})\right\|^{2} \\
&+\sum_{i=1}^{N}
q_{F}(F_{n,i}-F_{d})^{2}
+\sum_{i=1}^{N}
q_{I}\sigma_{F,i}^{2} \\
&+\sum_{i=0}^{N-1}
\left\|\mathbf{u}_{i}\right\|_{\mathbf{R}}^{2}
+\sum_{i=0}^{N-1}
\left\|\mathbf{u}_{i}-\mathbf{u}_{i-1}\right\|_{\mathbf{R}_{\Delta}}^{2}.
\end{aligned}
$$

各项含义：

- `Q_T`：切向路径跟踪；
- `q_N`：法向位置偏差，不宜过大，否则会压制力调节；
- `q_F`：法向力跟踪；
- `q_I`：力误差积分，降低稳态力偏差；
- `R`：控制能量；
- `R_Δ`：控制平滑性。

其中 $\mathbf{Q}_{T}$ 越大，MPC 越重视切向轨迹覆盖；$q_N$ 越大，MPC 越不愿偏离法向几何参考；$q_F$ 越大，MPC 越积极降低力误差；$q_I$ 用于惩罚持续力偏差；$\mathbf{R}$ 控制输入幅值，$\mathbf{R}_{\Delta}$ 控制输入变化率。实际调参时，$q_F/q_T$ 体现力优先程度，$q_N$ 则用于防止控制器为了追求力误差而完全放弃法向位置参考。

### 协同控制机理：MPC 控制为什么能实现力-位协同

MPC 可以看作“带预测时域的 QP 力-位协同控制”。单步 QP 只关心 `k+1` 时刻的位置和力，而 MPC 同时考虑未来 `N` 步内的轨迹误差、接触力误差、力边界和控制平滑性。因此，MPC 的协同机制不是当前一步的局部折中，而是预测时域内的整体折中。

从状态定义可以看出，MPC 把位置、速度、接触力和力误差积分放进同一个状态向量：

$$
\mathbf{s}_{k}
=
\begin{bmatrix}
\mathbf{x}_{k}^{T} &
\mathbf{v}_{k}^{T} &
F_{n,k} &
\sigma_{F,k}
\end{bmatrix}^{T}.
$$

这意味着优化器在预测未来状态时，不仅知道机器人会走到哪里，也知道这些运动会让接触力如何变化。状态更新中：

$$
\mathbf{x}_{k+1}
=\mathbf{x}_{k}+\Delta t\,\mathbf{v}_{k},
$$

$$
\mathbf{v}_{k+1}
=\mathbf{v}_{k}
+\Delta t\,\mathbf{M}_{d}^{-1}
(\mathbf{u}_{k}-\mathbf{D}_{d}\mathbf{v}_{k}),
$$

$$
F_{n,k+1}
=F_{n,k}
-\hat{K}_{e}\Delta t\,\mathbf{n}_{c}^{T}\mathbf{v}_{k+1}.
$$

前两行描述控制输入对位置运动的影响，第三行描述位置运动对接触力的影响。三者合在一起，就形成了“输入-运动-接触力”的预测链条。

MPC 目标函数同时惩罚未来切向位置误差、法向位置误差、法向力误差和力误差积分：

$$
\sum_{i=1}^{N}
\left\|\mathbf{P}_{T}(\mathbf{x}_{i}-\mathbf{x}_{d,i})\right\|_{\mathbf{Q}_{T}}^{2}
+\sum_{i=1}^{N}q_{N}
\left\|\mathbf{n}_{c}^{T}(\mathbf{x}_{i}-\mathbf{x}_{d,i})\right\|^{2}
+\sum_{i=1}^{N}q_{F}(F_{n,i}-F_{d})^{2}
+\sum_{i=1}^{N}q_{I}\sigma_{F,i}^{2}.
$$

其中切向位置项保障任务覆盖，力误差项保障接触安全和工艺要求，积分项减少持续偏差，法向位置项则防止控制器为了力误差完全放弃几何参考。由于这些项在同一个目标函数中求和，优化器会在整个预测窗口内寻找一组控制序列，使轨迹跟踪和接触力调节达到综合最优。

MPC 的力边界约束进一步使协同具有预测性：

$$
F_{min}-s_{i}^{-}
\leq F_{n,i}
\leq F_{max}+s_{i}^{+},
\qquad i=1,\ldots,N.
$$

这表示控制器不仅要避免当前时刻越界，还要避免未来若干步内按当前控制趋势发展后越界。例如，当参考轨迹即将进入高刚度区域时，MPC 可以提前减小法向压入速度或增大卸载趋势；单步 QP 通常只有在当前力已经接近边界时才会明显反应。

因此，MPC 的力-位协同机制可以概括为：

- 用统一状态 `s=[x,v,F_n,σ_F]` 同时描述运动状态和接触力状态。
- 用预测模型表达控制输入对未来位置和未来接触力的共同影响。
- 用时域目标函数同时惩罚轨迹误差、力误差、积分误差和控制代价。
- 用未来力边界约束提前抑制可能导致越界的运动。
- 每次只执行第一步控制，并在下一周期根据真实力反馈重新优化。

这使 MPC 成为比单步 QP 更强的通用优化基线。它能够实现力-位协同，而且协同具有前瞻性；但它也更依赖模型准确性和在线求解时间。柔性材料刚度估计错误、接触法向变化或求解器超时都会削弱这种协同效果，因此实物实验中必须报告计算时间、求解失败次数和 fallback 次数。

### 6.5 MPC 约束

力边界：

```text
F_min - s_i^- <= F_{n,i} <= F_max + s_i^+
s_i^- >= 0, s_i^+ >= 0
```

输入边界：

```text
||u_i||_∞ <= u_max
```

速度边界：

```text
||v_i||_∞ <= v_max
```

可选法向卸载约束：

```text
if F_{n,0} > F_max:
    n_c^T v_1 >= v_unload_min
```

该约束可避免力已经超上界时 MPC 继续沿法向压入。

### 6.6 滚动优化流程

每个控制周期：

1. 读取当前 `x,v,F_n,K_hat`；
2. 生成未来 `N` 步参考轨迹 `x_{d,1:N},v_{d,1:N}`；
3. 固定局部模型 `A,B,c`；
4. 求解 QP；
5. 执行第一步输入 `u_0`；
6. 将解序列左移作为下一周期 warm start；
7. 记录求解时间、状态和约束松弛量。

### 6.7 伪代码

```python
class LinearContactMPCController(ForcePositionControllerBase):
    def __init__(self, horizon, model_params, weights, limits, solver):
        self.N = horizon
        self.model = model_params
        self.weights = weights
        self.limits = limits
        self.solver = solver
        self.u_prev = np.zeros(3)
        self.warm_start = None

    def reset(self):
        self.u_prev[:] = 0.0
        self.warm_start = None

    def compute(self, obs, ref, contact, dt):
        n = normalize(contact.n)
        K_hat = clip(contact.K_hat, self.limits.K_min, self.limits.K_max)

        s0 = np.r_[obs.x, obs.v, contact.F_n, self.sigma_f]
        ref_horizon = build_reference_horizon(ref, self.N, dt)

        A, B, c = build_linear_contact_model(
            n=n,
            K_hat=K_hat,
            M_d=self.model.M_d,
            D_d=self.model.D_d,
            eps_f=self.model.eps_f,
            F_d=ref.F_d,
            dt=dt,
        )

        H, g, Aineq, bineq, Aeq, beq, lb, ub = build_mpc_qp(
            s0=s0,
            A=A,
            B=B,
            c=c,
            ref_horizon=ref_horizon,
            contact=contact,
            weights=self.weights,
            limits=self.limits,
            u_prev=self.u_prev,
        )

        result = self.solver.solve(
            H, g, Aineq, bineq, Aeq, beq, lb, ub,
            warm_start=self.warm_start,
        )

        if result.success:
            U = unpack_control_sequence(result.z, self.N)
            u = U[0]
            self.u_prev = u.copy()
            self.warm_start = shift_solution(result.z)
            status = "solved"
        else:
            u = fallback_impedance(obs, ref, contact, self.limits)
            self.warm_start = None
            status = "fallback"

        u = limit_norm(u, self.limits.u_max)

        debug = {
            "controller_name": "linear_contact_mpc",
            "solve_status": status,
            "horizon": self.N,
            "e_f": contact.F_n - ref.F_d,
            "u_norm": norm(u),
            "max_slack": result.max_slack if result.success else np.nan,
            "predicted_F_peak": result.predicted_F_peak if result.success else np.nan,
        }
        return u, debug
```

### 6.8 与旧 MPC 控制器的区别

本文建议的 MPC 是通用线性接触 MPC，具有以下特点：

- 不包含论文复现方法中的 `PSI`、障碍物距离、目标意图、轨迹重规划等共享控制专用变量；
- 不把人类输入作为 MPC 的参与者或博弈方；
- 只服务于执行层力-位控制；
- 状态、输入、代价和约束都能直接对应柔性接触扫描实验；
- 可以在无 RCM 和有 RCM 工况中复用同一套模型。

因此论文中可把它作为“标准优化控制基线”，而不是另一个复杂方法。

### 6.9 参数整定

推荐初始值：

```text
N = 5~15
dt = 0.01~0.02 s
q_T = 100~1000
q_N = 1~50
q_F = 10~500
q_I = 1~50
R = 1e-3~1e-1
R_delta = 1e-2~1
```

整定顺序：

1. 先关闭力约束，只保留 `q_T,q_F,R`，确认能跟踪和调力。
2. 加入力上下界软约束，并设置较大的 `w_slack`。
3. 加入 `R_delta`，降低控制跳变。
4. 调整 `N`。若求解时间过大，优先缩短时域，而不是降低采样周期稳定性。
5. 在实物中设置求解超时。超时后使用阻抗 fallback。

### 6.10 论文中可写的优缺点

优点：能统一表达未来轨迹、力误差、输入平滑和力边界；比单步 QP 更能处理参考变化；适合作为较强仿真基线。

不足：在线求解计算量更大；依赖局部线性模型和刚度估计；若柔性材料参数快速变化，预测模型会失配；实物部署需要 warm start、求解超时和 fallback。

## 7. 四类方法的统一实验接入

### 7.1 无 RCM 接入

无 RCM 表面扫描中，观测量直接来自工具尖端：

```python
obs.x = robot_state["tool_position"]
obs.v = robot_state["tool_position_velocity"]
```

参考量：

```python
ref.x_d = x_tool_ref
ref.v_d = xdot_tool_ref
ref.F_d = cfg.F_desired
```

接触量：

```python
contact.F = measured_or_virtual_force
contact.F_n = signed_normal_force
contact.n = surface_normal
contact.K_hat = est.K_hat
contact.B_hat = est.B_hat
contact.F_min = cfg.F_min
contact.F_max = cfg.F_max
```

控制器输出 `u_tool` 后，沿用现有流程：

```python
u_rot = compute_u_rotation(...)
u_cart = np.r_[u_tool, u_rot]
tau = J_tool.T @ u_cart
```

### 7.2 有 RCM 接入

有 RCM 长工具实验中，有两种实现方式。

第一种是“控制器仍在工具尖端空间计算，后处理映射到法兰”。流程为：

```text
tool 参考 x_tool_ref
    -> controller 得到 u_tool
    -> RCM 几何或雅可比近似映射到 u_flange
    -> J_flange^T 映射为关节力矩
```

第二种是“先将工具参考映射到法兰参考，控制器在法兰空间计算”。这与现有 `compute_torque_with_rcm(...)` 更接近：

```text
x_tool_ref, xdot_tool_ref
    -> tool_to_flange_full_ref(...)
    -> x_flange_ref, xdot_flange_ref
    -> controller 使用 flange position error
    -> u_flange
    -> J_flange^T 映射为关节力矩
```

为了减少改动，建议优先采用第二种。四类通用控制器不需要知道 RCM 细节，只需要输入 `obs.x=flange_pos`、`ref.x_d=x_flange_ref`。接触力误差仍使用工具尖端法向力。

论文中应说明：RCM 是实验中的几何边界条件，控制器基线仍然是同一类力-位执行层方法。

## 8. 评价指标与日志字段

四类基线和本文合作博弈方法必须使用同一指标。

力安全：

```text
E_F^RMS = sqrt(1/N Σ_k (F_{n,k}-F_d)^2)
E_F^max = max_k |F_{n,k}-F_d|
T_vio = dt Σ_k 1[F_{n,k}<F_min or F_{n,k}>F_max]
J_F = sqrt(1/(N-1) Σ_k ((F_{n,k+1}-F_{n,k})/dt)^2)
```

轨迹质量：

```text
E_T^RMS = sqrt(1/N Σ_k ||P_T(x_k-x_{d,k})||^2)
E_N^RMS = sqrt(1/N Σ_k ||P_N(x_k-x_{d,k})||^2)
J_x = RMS jerk or RMS acceleration variation
```

几何约束，仅 RCM 工况使用：

```text
E_gc^RMS = sqrt(1/N Σ_k e_{gc,k}^2)
E_gc^max = max_k e_{gc,k}
```

实时性：

```text
T_comp^mean = mean solve_time
T_comp^max = max solve_time
N_fail = number of solver fallback events
```

建议日志字段：

```text
t
controller_name
x, v, x_d, v_d
F_n, F_d, F_min, F_max
e_t_norm, e_n, e_f
u_x, u_y, u_z, u_norm
tau_1 ... tau_7
K_hat, B_hat
solve_status, solve_time
slack_low, slack_high
e_gc
```

## 9. 对比方法在论文中的写法

第三章控制器对比建议按以下顺序介绍：

1. 固定笛卡尔阻抗控制：说明接触任务中最基础的柔顺控制基线；
2. 混合力/位控制：说明传统显式方向分离方法；
3. 单步 QP：说明约束优化型基线；
4. 线性 MPC：说明有限时域优化型基线；
5. 本文合作博弈方法：说明通过 `alpha_FP` 和帕累托反馈实现动态力-位折中。

论文中应避免把所有基线都写成“很差”。更稳妥的对比表述如下：

| 方法 | 优势 | 局限 | 预期实验现象 |
|---|---|---|---|
| 固定阻抗 | 实时、稳定、易部署 | 固定参数难适应变刚度 | 单段材料稳定，硬区力峰值或软区轨迹漂移 |
| 混合力/位 | 方向分工清楚 | 依赖法向和选择矩阵 | 法向力较好，但切换和耦合处可能抖动 |
| 单步 QP | 约束清晰、实现简单 | 无长时域预测 | 越界减少，但对刚度估计敏感 |
| 线性 MPC | 可预测未来轨迹和约束 | 计算量大、模型依赖强 | 仿真表现较强，实物需降频或 fallback |
| 合作博弈 | 闭式反馈、动态折中、实时性强 | 需要设计权重和增益表 | 综合指标稳定，不保证所有单项最优 |

## 10. 最小实现顺序

若时间紧张，建议按以下顺序实现：

1. 固定阻抗控制器：最容易实物跑通，可作为安全 fallback。
2. 混合力/位控制器：验证切向位置、法向力分离基线。
3. 单步 QP 控制器：先在仿真跑通力边界约束，再决定是否上实物。
4. 线性 MPC 控制器：优先作为仿真强基线；实物中可降到 50 Hz 或只做离线对比。

最小代码路径：

```text
common.py
    Observation, Reference, ContactState, ControllerLimits
    normalize, limit_norm, build_projectors

impedance_controller.py
    CartesianImpedanceController

hybrid_force_position_controller.py
    HybridForcePositionController

qp_controller.py
    SingleStepQPController

mpc_controller.py
    LinearContactMPCController
```

最小实验路径：

```text
仿真:
    impedance vs hybrid vs QP vs MPC vs 本文方法

无 RCM 实物:
    impedance vs hybrid vs 本文方法
    QP 可选

有 RCM 实物:
    impedance vs 本文方法
    hybrid 可选
    MPC 建议不强制
```

## 11. 推荐章节落点

第二章可补充阻抗控制、混合力/位控制和优化控制基础，给出通用模型即可。

第三章实验小节中可写：

```text
为验证所提合作博弈力-位协同控制器的综合折中能力，本文选取固定笛卡尔阻抗控制、混合力/位控制、单步 QP 控制和线性 MPC 控制作为对比方法。四类方法均基于同一柔性接触模型和同一实验输入，不采用任务专用学习模型或论文复现式共享控制器，以保证比较集中于力-位执行层控制能力。
```

第五章综合实验中可写：

```text
第五章不再引入新的执行层控制律，而是在统一平台中调用第三章控制器与第四章仲裁器。对比控制器仍采用第三章定义的通用基线，从而保证无 RCM 和有 RCM 工况下的实验差异来自任务边界，而不是来自基线方法定义变化。
```

## 12. 实现注意事项

1. 坐标符号必须先统一。建议所有控制器内部使用 `F_n>0` 表示压向材料的接触力大小，`e_f=F_n-F_d` 表示力过大为正。
2. 法向方向必须写入日志。若出现控制方向错误，优先检查 `n_c`、`unload_dir` 和力传感器符号。
3. QP/MPC 的 `K_hat` 必须限幅。低估刚度会导致过度压入，高估刚度会导致过度保守。
4. 所有控制器使用同一硬安全监督层。不要让某个方法因没有安全层而显得更差，也不要让某个方法因额外安全层而不公平。
5. QP/MPC 必须有 fallback。实物控制中求解失败不能直接发送上一帧未知控制量。
6. 比较时固定参考轨迹、材料、力目标、采样频率和统计时间窗。接触建立阶段不纳入正式统计。
7. 若 MPC 无法稳定达到 100 Hz，可明确写为 50 Hz 优化基线，并报告计算时间。这本身就是对比结论的一部分。

## 13. 总结

四类通用基线的定位可以概括为：

- 固定阻抗：最基础的柔顺接触控制，体现固定参数折中的局限；
- 混合力/位：最经典的方向分离力控方法，体现显式切向/法向分工的优缺点；
- 单步 QP：最简洁的在线约束优化方法，体现力边界与位置目标的即时折中；
- 线性 MPC：有限时域优化方法，体现预测约束控制的性能与实时性代价。

这四类方法与本文合作博弈控制器形成清楚对比：前两类强调工程直观性，后两类强调在线优化表达，本文方法则强调闭式反馈、动态优先级和实时综合折中。这样的基线体系能直接回应 6 月 17 日修改意见中关于“为什么柔性操作必须做力-位协同”以及“为什么合作博弈相较阻抗、QP、MPC 具有必要性”的要求。
