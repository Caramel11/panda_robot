# Chapter 5 Dual-Arbitration Simulation Results

## Gazebo Probe

- status: `unreachable`
- detail: ROS master is not reachable from this shell.

## Metric Summary

| method | score | force RMS / N | force peak / N | violation / s | task RMS / mm | accept | suppress | alpha range | corr(alpha,Khat) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| fixed_08 | 28.495 | 0.465 | 1.192 | 20.10 | 2.57 | 0.58 | 0.42 | [0.80, 0.80] | 0.00 |
| fixed_05 | 8.386 | 0.317 | 1.754 | 4.50 | 2.52 | 0.58 | 0.42 | [0.50, 0.50] | 0.00 |
| fixed_02 | 4.735 | 0.204 | 2.276 | 1.54 | 2.49 | 0.58 | 0.42 | [0.20, 0.20] | 0.00 |
| hr_only | 8.001 | 0.315 | 1.751 | 4.50 | 1.74 | 0.89 | 0.91 | [0.50, 0.50] | 0.00 |
| fp_only | 4.320 | 0.235 | 1.530 | 1.52 | 5.63 | 0.00 | 1.00 | [0.19, 0.60] | -0.97 |
| dual_arbitration | 3.843 | 0.230 | 1.634 | 1.48 | 1.74 | 0.89 | 0.93 | [0.19, 0.60] | -0.97 |

## Key Figures

![Metric bars](metrics_bars.png)

![Force alpha stiffness](force_alpha_stiffness_timeseries.png)

![XY trajectory](trajectory_xy.png)

![Unsafe normal push](unsafe_normal_push_detail.png)

![Dual alpha khat response](dual_alpha_khat_response.png)

## Main Observation

The dual method keeps the reference-level human correction and the execution-level force-position arbitration separated.  It accepts tangential human corrections, suppresses unsafe normal push, and changes alpha_fp with stiffness and force margin. This is why it achieves a lower composite score than the best fixed-alpha baseline.

- best fixed score: 4.735
- dual score: 3.843
- score margin: 0.892
