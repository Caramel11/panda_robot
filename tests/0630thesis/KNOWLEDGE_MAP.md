# 0608 thesis knowledge map

This note records the working understanding used for later thesis writing,
code changes, and debugging.  It intentionally links the thesis text, the
chapter code, and the reference thesis writing pattern.

## 1. Repository and source boundaries

- Parent workspace: `/home/liu/franka_ws_1101`.
- Controller/thesis work lives in the child repository `src/panda_robot`.
- Do not treat `src/panda_robot` files as ordinary parent-repository files.
- Generated data such as `*.npz`, `*.npy`, figures under result folders, cache
  files, and build outputs should not be committed unless explicitly reviewed.

## 2. Current thesis structure

Entry point:

- `main_ustc.tex` is the USTC thesis entry.
- `ustcsetup.tex` currently sets `tocdepth=2`, so subsections appear in the
  table of contents.
- Chapters are included with `\input`, which avoids the earlier `\include` and
  `latexmk -outdir` auxiliary-file issue.
- Current compiled PDF:
  `src/panda_robot/tests/0608thesis/compiled_pdfs/main_ustc.pdf`, 99 pages.

Chapter logic:

- Chapter 1: motivation, literature, research contents, organization.
- Chapter 2: preliminaries; robot kinematics/dynamics, constrained motion and
  contact modeling, optimal control and cooperative differential games,
  stability/robustness tools.
- Chapter 3: rigid geometric constraint task.  Core symbol:
  `alpha_HR`, meaning human-robot supervision arbitration.
- Chapter 4: flexible contact task.  Core symbol: `alpha_FP`, meaning
  force-position priority.  It is not the same as `alpha_HR`.
- Chapter 5: industrial constrained contact task.  Core idea: two-layer
  arbitration.  `alpha_HR` acts only on reference generation, while
  `alpha_FP` acts only on execution-level force-position scheduling.
- Chapter 6: summary and outlook.

Current chapter lengths make Chapter 4 the most detailed technical chapter,
with Chapter 5 close behind.  Chapters 3, 4, and 5 already use problem,
definition, assumption, lemma, theorem, and proof modules.

## 3. Reference thesis writing pattern

Reference PDF:
`attachments/reference_pdfs/非接触人机交互场景下基于微分博弈的协同控制策略研究_童康.pdf`.

Structural pattern:

- Six chapters: introduction, preliminaries, three technical chapters, summary.
- Chapter 2 prepares exactly the theory needed later, rather than becoming a
  general textbook.
- Technical chapters use the same arc:
  problem and definitions -> method/controller design -> theoretical analysis
  -> simulation -> experiment -> chapter summary.
- Each technical chapter starts by identifying the previous chapter's limitation
  and then states the new difficulty.
- Chapter summaries are compact but complete: method proposed, theory proved,
  simulations/experiments verified, limitation or next step implied.

Figure/table pattern:

- Introduction and preliminaries use illustrative figures to orient the reader.
- Method chapters begin with a framework/block diagram.
- Simulation and experiment sections concentrate trajectory, error, platform,
  scene, and metrics figures.
- Tables are used for repeated-trial statistics, parameter settings, platform
  specifications, and method comparisons.
- Reference thesis is about 97 pages.  Its figure list has roughly 25 figures;
  Chapter 5 is the densest chapter, with safety scenario figures and repeated
  experiment tables.

Style pattern:

- The tone is rigorous but readable: explain why a model is introduced before
  giving formulas.
- Definitions and assumptions are followed by intuitive explanations when the
  condition might feel abstract.
- Avoid jumping directly from a formula to a result table.  Insert transition
  paragraphs that say what the formula means in the task.
- Use phrases like "与上一章不同", "在控制目标明确后", "为了进一步说明",
  "上述结果表明" to carry the reader across sections.
- A good paragraph often has this shape: scene or difficulty -> mathematical
  object -> engineering interpretation.

## 4. Chapter 3 and Chapter 4 code map: `tests/0213`

Main roles:

- `real_GT_KF.py`: game-theoretic shared control with fuzzy/KF arbitration.
- `real_GT_Sigmoid.py`: GT controller with sigmoid-style arbitration baseline.
- `real_MPC_KF.py`: MPC/shared-control baseline with fuzzy/KF arbitration.
- `real_MPC_Sigmoid.py`: MPC baseline with sigmoid arbitration.
- `real_GT_KF_with_other_target.py`: other-target variant.
- `arbitrary.py`: large support file for robot state update, trajectory
  deformation, RCM mapping, MoveIt integration, shared-control MPC/Pareto
  solver, goal intent, callbacks, and logging.
- `fuzzy_logic.py`: fuzzy logic for `lambda` and `delta_lambda`; includes
  IF-ELSE baseline.
- `kalman_filter.py`: `DeltaLambdaUpdater` and `KalmanFilterFusion`.
- `lqr.py`: computes `K_gt` as a function of `alpha`.
- `data_analysis_4method.py`: four-method statistical analysis and figure.

Important code facts:

- `fuzzy_logic.py` maps human force, interaction time, and robot distance to
  `lambda`; derivative features feed `delta_lambda`.
- `kalman_filter.py` fuses direct `lambda` and integrated `delta_lambda` into a
  smoothed bounded arbitration parameter.
- `lqr.py` currently performs database generation at import time and writes
  `k_gt_database.npy`.  Be careful when importing or testing it.
- `data_analysis_4method.py` computes metrics:
  `mean_error_rcm`, `mean_error_ee`, `mean_force_norm`,
  `Efs_force_smoothness`, `traj_rms_jerk`, and `mean_dist_to_object`.
- Four-method comparison is mainly:
  `GT_KF`, `GT_Sigmoid`, `MPC_KF`, and `MPC_Sigmoid`.

Thesis mapping:

- Chapter 3 should treat `GT_KF` as the main method evidence for dynamic
  human-robot arbitration.
- Chapter 3 metrics should emphasize RCM error, EE tracking error, force,
  smoothness, jerk, and object distance.
- Chapter 4 has a more developed force-position theory in the thesis than the
  older `0213` code directly implements.  Reuse `0213` for the shared-control,
  arbitration, RCM, and metric language; use newer Chapter 4/5 code for
  force-position contact claims where available.

## 5. Chapter 5 code map: `tests/0608controller`

Main command from the local README:

```bash
cd /home/liu/franka_ws_1101
MPLCONFIGDIR=/tmp/matplotlib-codex \
python3 src/panda_robot/tests/0608controller/run_ch5_dual_arbitration.py \
  --backend gazebo --tune --seed 11
```

The script always runs the deterministic analytic simulation.  With
`--backend gazebo`, it also probes the current ROS/Gazebo master and writes
diagnostics.

Main methods:

- `fixed_08`, `fixed_05`, `fixed_02`: fixed force-position priority.
- `hr_only`: dynamic reference-level human-robot arbitration only.
- `fp_only`: dynamic execution-level force-position arbitration only.
- `dual_arbitration`: both layers enabled.

Core functions:

- `environment_stiffness`: piecewise stiffness profile.
- `human_delta`: scripted safe tangential correction plus unsafe normal push.
- `project_reference`: clamps unsafe normal reference using estimated stiffness
  and force limits.
- `compute_alpha_hr`: reference-level human influence scheduling.
- `compute_alpha_fp`: execution-level force-position priority scheduling from
  force margin, stiffness estimate, and tangent error.
- `metrics_for`: score and thesis-facing metrics.
- `plot_results`: produces Chapter 5 figures.

Latest validated result:

`src/panda_robot/tests/0608controller/results/ch5_dual_arbitration_20260608_215915`

Key validated numbers:

- Best fixed score: `fixed_02 = 4.735`.
- Dual-arbitration score: `3.843`.
- Score margin: `0.892`.
- Dual method: force RMS `0.230 N`, violation time `1.48 s`, task RMS
  `1.74 mm`, tangent acceptance `0.89`, normal suppression `0.93`,
  alpha range `[0.19, 0.60]`, corr(alpha, Khat) `-0.97`.

Thesis interpretation:

- The dual method keeps useful tangential human correction.
- It suppresses unsafe normal push before it reaches the execution layer.
- It lowers `alpha_FP` under high stiffness or low force margin, making the
  controller more force-prioritized.
- It outperforms the best fixed-alpha baseline because it avoids a single
  compromise parameter.

## 6. Debugging rules for later work

For `0213`:

- First verify whether a change affects real robot/MoveIt execution or only
  offline analysis.
- Avoid importing `lqr.py` casually because it writes `k_gt_database.npy`.
- For arbitration issues, log and compare raw fuzzy `lambda`, integrated
  `delta_lambda`, KF output, and final control gain.
- For result issues, compare RCM error, EE error, force norm, force smoothness,
  trajectory jerk, and distance to object before changing controller logic.

For `0608controller`:

- Start with the analytic backend.  It is deterministic and thesis-figure
  oriented.
- Use the latest validated result directory as the baseline unless a newer run
  is intentionally created.
- When tuning Chapter 5, check whether improvement comes from force RMS,
  violation time, tangent acceptance, normal suppression, or task RMS.  Do not
  rely only on composite score.
- Treat Gazebo status as diagnostic context.  A failed probe does not invalidate
  the analytic simulation; it may only mean ROS master is unreachable from the
  shell or the Gazebo model is stale.

For thesis writing:

- Keep `alpha_HR` and `alpha_FP` physically separate.
- Put a short engineering explanation after every abstract theorem/assumption.
- Tie every simulation figure to one sentence of verification purpose before
  the figure and one sentence of conclusion after it.
- Use the reference thesis chapter rhythm: previous limitation -> new problem
  -> formal definition -> method -> theory -> simulation/experiment -> summary.
- When adding figures, prefer framework diagram near the method section and
  metrics/trajectory/time-series figures near validation sections.
