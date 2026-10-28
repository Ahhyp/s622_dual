# C4.3 S4 W1: controller-side transport status.
# Sampled every control cycle (125 Hz) and published decimated (~10 Hz).
string state                  # IDLE | RUNNING | HOLD | ABORT | FAULT | COMPLETE
string failure_code           # "" when healthy; otherwise a §4.7.1/§4.8 code
bool   active                 # a trajectory is latched
bool   committed              # the last control cycle committed its increment
float64 t_traj                 # trajectory clock [s], frozen while holding
float64 t_traj_duration        # total trajectory duration [s]
float64[12] q_state
float64[12] q_cmd
float64[12] dq
int32  solve_fail_streak
int32  active_limit_count
float64 min_limit_margin_rad
float64 e_rel_pos_m
float64 e_rel_rot_rad
float64 e_obj_pos_m
float64 e_obj_rot_rad
float64 sigma_min_stack
float64 anchor_pos_gap_m
float64 anchor_rot_gap_rad
float64 qp_solve_time_ms

# ---- S4 W1 review (A): honest protection statement -------------------------
# "PARTIAL(e1,...) degraded(e2) missing(e8)" -- never claims completeness while
# a criterion is not actually detected by this controller.
string protection
bool   protection_complete
# Executor heartbeat: -1.0 means "never received".
float64 executor_heartbeat_age_s
bool    executor_heartbeat_seen
bool    executor_heartbeat_required
# B: standing-configuration check at start_transport time.
float64 start_pose_deviation_rad
bool    start_pose_ok

# ---- C7-A Rev.2: hold-request ack -------------------------------------------
# Echo of the last consumed hold request.  seq==0 -> no ack so far.
uint32 hold_request_seq
uint8  hold_request_result   # 0 none / 1 applied / 2 refused
string hold_origin           # "" / "controller" / "executor"

# ---- W3-R2 (approved D5-1): task-phase observation ---------------------------
# phase_index == -1 means "no trajectory latched".  The name is the frozen spec
# name of the phase containing t_traj; phase_elapsed_s is measured INSIDE the
# phase and phase_duration_s is that phase's frozen duration.  Consumers must
# treat a phase jump of more than +1 as a desync, never as progress.
int32  phase_index
int32  phase_count
string phase_name
float64 phase_elapsed_s
float64 phase_duration_s
