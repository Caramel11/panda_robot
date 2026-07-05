# 第 3/4/5 章控制实验扩展记录字段说明

本文档说明 `shared_gt_gazebo_node.py` 写入 `result.npz` 的扩展字段。该记录方案用于第 3、4、5 章快速对照、消融实验和真实环境噪声模拟实验，目标是把控制器输入、环境、误差、仲裁变量和力矩分解完整保存，便于后续绘图、指标重算和异常溯源。

## 基础实验信息

这些信息保存在每次实验的 `summary.json` 中：

- `task_mode`：`no_rcm` 或 `with_rcm`。
- `controller_variant`：参考层或对照控制器配置。
- `arbitration_strategy`：第 4/5 章参考层仲裁方法。
- `execution_strategy`：第 3/5 章执行层力位控制方法。
- `execution_alpha_profile`：执行层 `alpha_FP` 调节律参数组。
- `scenario`：轨迹或场景名称。
- `task_duration_s`、`dt`、`control_rate_hz`：任务时长和控制频率。
- `trajectory_scale`：轨迹缩放系数。
- `force_desired`、`force_min`、`force_max`、`force_margin`：接触力目标和安全边界。
- `contact_stiffness_hat`：名义环境刚度估计。
- `realistic_env_enabled`、`realistic_profile`、`realistic_seed`、`realistic_tau_noise_gain`：真实环境噪声模拟配置。
- `human_input_profile`、`external_human_scale`、`external_human_max_delta_m`：人类输入生成或外部输入配置。
- `gravity_compensation_scale`、`max_tau_rate`、`max_tau_abs`：力矩输出约束。

## 轨迹、人类输入与参考层

每个采样点保存：

- `nominal_ref_*`：自主参考轨迹。
- `human_ref_*`、`x_h_*`、`x_h_safe_*`：人类候选参考与安全投影后的参考。
- `x_tele_*`、`x_reshape_*`、`x_reshape_vel_*`：直接遥操作参考和 0213 轨迹变形参考。
- `human_button`、`human_force_cmd_N`、`human_raw_force_N_*`、`human_delta_cmd_*`、`human_event_type_id`：模拟真实主端输入的按钮状态、输入幅值和事件类别。
- `alpha`、`alpha_raw`、`alpha_dynamic`、`delta_lambda`、`beta`、`rho_F`、`D_r`、`D_c`、`kappa_N`：第 4 章参考层仲裁相关变量。

## 执行层力位控制

第 3/5 章本文方法和固定 alpha 消融会额外保存：

- `alpha_FP`、`alpha_FP_target`：最终执行层力位优先级。
- `alpha_FP_fuzzy_base`、`alpha_FP_after_margin`、`alpha_FP_after_tracking`、`alpha_FP_after_force_risk`、`alpha_FP_after_stiffness`、`alpha_FP_safe_clipped`：`alpha_FP` 从模糊基础值到力安全、跟踪、刚度调节后的中间量。
- `alpha_FP_margin_gate`、`alpha_FP_force_gate`、`alpha_FP_track_gate`、`alpha_FP_tracking_boost_gate`、`alpha_FP_risk_margin_gate`、`alpha_FP_stiffness_gate`、`alpha_FP_stiffness_transition_gate`：各调节因素的门控强度。
- `alpha_FP_F_h`、`alpha_FP_S_h`、`alpha_FP_D_r`、`alpha_FP_rho_F`、`alpha_FP_s_F`：force margin 调节律的输入特征。
- `alpha_FP_K_hat`、`alpha_FP_dK`：刚度估计及变化率。
- `execution_e_r_*`、`execution_e_v_*`、`execution_e_f_*`：位置、速度和力误差向量。
- `execution_K_axes_*`：不同轴向、不同误差项的增益表查询结果。
- `execution_component_position_*`、`execution_component_velocity_*`、`execution_component_force_*`、`execution_component_sigma_*`：控制输出中位置、速度、力误差积分项的分解。
- `execution_u_game_raw_*`、`execution_u_tracking_boost_*`、`execution_u_pre_limit_*`：限幅前的执行层输出。
- `execution_output_limit_active`、`execution_output_limit_scale`：执行层限幅是否触发及缩放比例。

## 任务误差与力反馈

- `tracking_error`、`rcm_error`：末端跟踪误差和 RCM 约束误差。
- `task_pos_err_*`、`task_vel_err_*`：任务空间位置和速度误差。
- `task_tangential_pos_err_norm`、`task_normal_pos_err`：切向位置误差和法向位置误差。
- `task_tangential_vel_err_norm`、`task_normal_vel_err`：切向速度误差和法向速度误差。
- `task_force_error_N`、`task_force_error_ratio`、`task_force_span_N`：接触力误差和归一化力误差。
- `y_f`、`y_f_candidate`、`y_f_safe`：参考层预测力、安全候选力和投影后的安全力。
- `y_f_actual_true`、`force_feedback_raw`、`force_feedback_clipped`、`force_feedback_control`：根据实际末端压入量生成的真实接触力代理和进入控制器的滤波反馈。

## 真实环境噪声与刚度区

真实环境模拟开启时保存：

- `F_measured`、`F_raw`、`F_true_normal`、`Fx`、`Fy`、`Fz`、`Mx`、`My`、`Mz`：六维力传感器模拟通道。
- `K_env_realistic`、`K_env_actual`、`K_env_reference`：实际末端和参考点对应的局部刚度。
- `B_env_realistic`、`B_env_reference`：局部阻尼。
- `stiffness_transition_s`、`stiffness_transition_reference_s`：软硬海绵边界平滑过渡参数。
- `sponge_local_y_m`、`sponge_local_y_actual_m`、`sponge_local_y_reference_m`：相对于刚度分界线的局部坐标，`y>0` 为软区，`y<0` 为硬区。
- `actual_normal_down_m`：实际末端相对于参考面的法向压入量。
- `u_tool_noise_*`、`wrench_noise_norm`：工具扰动和传感器噪声强度。

## 力矩与限幅

- `tau_*`：最终发送给 Gazebo effort 控制器的关节力矩。
- `tau_task_*`：任务空间控制贡献转换后的关节力矩。
- `tau_gravity_*`：Pinocchio 重力项。
- `tau_pre_realistic_*`：加入重力项后的原始力矩。
- `tau_after_realistic_*`：经过真实环境力矩扰动后的力矩。
- `tau_realistic_delta_*`：真实环境扰动量。
- `tau_rate_clip_delta_*`：关节力矩速率/幅值限幅造成的修正量。
- `tau_pos_component_*`、`tau_rot_component_*`、`tau_tool_aux_component_*`、`tau_rcm_component_*`：位置、姿态、工具辅助和 RCM 投影力矩分解。
- `tau_task_norm`、`tau_gravity_norm`、`tau_pre_realistic_norm`、`tau_after_realistic_norm`、`tau_command_norm`：关键力矩模长。

## 后续绘图建议

- 证明 `alpha_FP` 刚度敏感性时，应联合绘制 `K_env_realistic`、`alpha_FP_after_stiffness`、`alpha_FP_stiffness_gate`、`task_force_error_N` 和 `task_tangential_pos_err_norm`。
- 分析控制器不稳定时，应先查看 `execution_output_limit_active`、`u_pos_limit_active`、`tau_rate_clip_delta_*` 和 `tau_realistic_delta_*`。
- 比较真实环境噪声影响时，应联合绘制 `F_true_normal`、`F_measured`、`F_estimated`、`wrench_noise_norm` 和 `actual_normal_down_m`。
