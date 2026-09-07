# Project History

## Stage 0: Preliminary State-Based BC Attempt (December 2025)

**Problem:**
An initial behavior cloning model was implemented and tested. The model used a 13-dimensional state vector (position + orientation + linear velocity + angular velocity) as input and a 3-dimensional velocity vector as output. The model was trained on data recorded from CERLAB autonomous flight sessions using `record_expert.py` in `uav_bc_tools`.

The resulting flight behavior was unstable. The policy failed to generalize and did not produce reliable navigation behavior in Gazebo.

**Decision:**
The state-based BC approach is insufficient as a formal baseline because:
1. It does not reflect the actual deployment constraint (front camera only).
2. The state vector encodes privileged information (absolute position, global orientation) not available at test time on a real platform.
3. Its poor performance cannot be cleanly attributed to architecture or data — the input representation itself is mismatched with the research goal.

The BC baseline must be rebuilt using front-camera images as input, trained on newly collected CERLAB teacher demonstrations, under the same protocol as all KD methods.

**Implementation:**
- Training scripts: `legacy/old_bc_training/train_bc*.py`
- Expert data: `legacy/old_bc_training/data/expert_20251219_*.npz` (state-based, no images)
- Saved models: `legacy/old_bc_training/models/` (do not reload)

**Observation:**
Old expert `.npz` files contain keys: `t`, `position`, `orientation`, `lin_vel`, `ang_vel`, `action`, `action_source`. No image key is present. The old data cannot be used to train the image-based formal baseline.

**Next Step:**
Reproduce CERLAB teacher automatic flight in Gazebo. Verify all required ROS topics are published. Collect synchronized teacher demonstrations with camera images.

---

## Stage 1: Project Restructure (2026-06-28)

**Problem:**
Project files were scattered across multiple directories with no unified structure. The `uav_kd_project/` directory contained only CLAUDE.md. The old BC work and the new KD project were not cleanly separated.

**Decision:**
Consolidate all future KD project files under `/home/l/uav_kd_project/`. Archive the old state-based BC work in `legacy/`. Keep CERLAB as an external dependency in `/home/l/cerlab_ws/`. Keep `uav_simulator` in `/home/l/catkin_ws/src/uav_simulator/` as an upstream dependency.

**Implementation:**
- Created full directory skeleton under `uav_kd_project/`
- Copied old BC scripts, data, models, tools, and figures into `legacy/`
- Symlinked existing rosbags into `data/raw/candidate_bags/` (693 MB, not duplicated)
- Created `README.md`, `legacy/README.md`, `docs/00_project_history.md`, `docs/01_environment_log.md`
- No files deleted. No files moved. CERLAB untouched.

**Observation:**
The project now has a clean, thesis-ready structure. The legacy archive makes the project history explicit and separable from the formal experimental pipeline.

**Next Step:**
Phase 1 — Reproduce CERLAB teacher automatic flight in Gazebo. Verify topics. Record `docs/02_topic_mapping.md`.

---

## Stage 2: BC Baseline Closed Out — Rotation Data Audit, v002 Rebuild, Deployment Test (2026-07-01)

**Problem:**
The rebuilt BC baseline (Phase 5, trained on `uav_kd_v001`) showed near-zero yaw_rate MSE, but this was a false signal: `uav_kd_v001`'s val split (`ep_test_005` alone) was 100% straight-line corridor flight with zero rotation samples, so the model could trivially predict 0 and appear to "succeed." Separately, CERLAB's own turning behavior in this straight-walled corridor is not smooth cornering — it is achieved through brief in-place spot-turns (yaw rotation with near-zero translation), typically during dynamic-obstacle avoidance.

**Decision:**
Rather than collect entirely new episodes, first audit whether sufficient rotation signal already existed in already-recorded-but-unused data. Two episodes (`ep_006`, held back from v001) and two newly recorded episodes (`ep_008`, `ep_009`) were confirmed to contain real spot-turn segments (visually verified via motion-blur inspection — see `docs/04_experiment_log.md`, 2026-07-01 entries). Rebuild the dataset (`uav_kd_v002`) merging these in and restructuring the val split so it also contains rotation samples, rather than recording fresh data first.

**Implementation:**
- `kd_uav/datasets/dataset_builder.py`: added `traj_magnitude_threshold` filter — during sustained spot-turns, `bspline_trajectory` is not republished (UAV isn't translating), so all K trajectory waypoints freeze near the UAV's own position; the existing `valid_traj` check (`dx0>=0`) missed this. Now `valid_traj` also requires the trajectory to be non-degenerate.
- Built `data/processed/uav_kd_v002/` (train=10170 from `ep_003/004/007/008/009`, val=1531 from `ep_005/006`; val now ~11.4% rotation samples vs. 0% in v001).
- Retrained BC (`configs/train/bc_v002.yaml` → `runs/bc_v002/20260701_142506/`) as a sanity check.
- Deployed `best_model.pt` in Gazebo (`scripts/launch_student_bc.sh`) for a live flight test.

**Observation:**
- BC(v002) val loss (0.0526) is not directly comparable to v001's (0.0116) — the harder, rotation-inclusive val set explains most of the gap, not a regression.
- yaw_rate MSE (0.0879) now beats both a "predict 0" baseline (0.1177) and a "predict val-mean" baseline (0.0999) — the model has learned genuine (if modest) rotation signal, no longer degenerate.
- vx MSE (0.1014) is *worse* than a "predict val-mean" baseline (0.0899) — a real episode-level generalization gap on the fully-held-out `ep_006`, not something the rotation fix caused; noted as an open issue, not blocking.
- Gazebo deployment of BC(v002) ran stably for 2+ minutes with no crash: cmd_vel showed real, varying yaw_rate (not stuck at 0), including several `vx≈0 & yr large` moments consistent with learned spot-turn behavior.

**Next Step:**
Baseline phase considered closed. Proceed to TrajKD (Option B, per `CLAUDE.md` §11), using `uav_kd_v002` as the standard dataset and BC(v002)'s results (val loss 0.0526, yaw_rate MSE 0.0879) as the fair comparison baseline for all subsequent KD ablations — not the old v001/BC-RedWP numbers, which were measured on an easier, rotation-free validation set.
