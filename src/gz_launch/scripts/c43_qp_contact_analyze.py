#!/usr/bin/env python3
"""Report controller metrics separately from native physical-contact audits."""
import json
from pathlib import Path
import sys
import numpy as np


def analyze(run):
    summary=json.loads((run/'summary.json').read_text())
    rows=[json.loads(line) for line in (run/'qp_status.jsonl').open()]
    active=[r for r in rows if r['state']=='RUNNING' and r['sim'] is not None]
    metadata_path=run/'metadata.json'
    metadata=json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
    dt=metadata.get('qp_config_dt_s',.008)
    metric=dict(error=summary['error'],status_rows=len(rows),states=sorted({r['state'] for r in rows}),
        formal_w7_complete=False,source='QP controller estimates; not Gazebo object tracking error')
    if len(active)>2:
        x=np.array([r['sim'] for r in active]);y=np.array([r['t_traj'] for r in active])
        slope=float(np.polyfit(x,y,1)[0]);metric['controller_traj_seconds_per_sim_second']=slope
        metric['trajectory_clock_rate_error_percent']=100.*(slope-1.)
        metric['effective_controller_updates_per_sim_second_inferred']=slope/dt
        metric['qp_config_dt_s']=dt
        metric['physics_step_s']=metadata.get('physics_step_s')
        metric['frequency_measurement_note']='inferred from RUNNING t_traj progression and configured dt; status messages are decimated, not per-cycle RT timing'
        if all(r.get('mono') is not None for r in active):
            wall=np.array([r['mono'] for r in active])
            wall_slope=float(np.polyfit(wall,y,1)[0])
            metric['controller_updates_per_wall_second_inferred']=wall_slope/dt
            metric['sim_seconds_per_wall_second_during_run']=float(np.polyfit(wall,x,1)[0])
        metric['controller_peaks']={k:max(r[k] for r in active) for k in
            ('e_rel_pos_m','e_rel_rot_rad','e_obj_pos_m','e_obj_rot_rad','anchor_pos_gap_m','qp_solve_time_ms')}
        if metric['controller_peaks']['qp_solve_time_ms']==0:
            metric['controller_peaks']['qp_solve_time_ms']=None
            metric['solver_timing_note']='not instrumented; zero status field is not a measured zero solve time'
    metric['protection_reports']=sorted({r['protection'] for r in rows})
    metric['protection_complete_in_all_reports']=all(r['protection_complete'] for r in rows)
    for name in ('qp_smoke_result','qp_carry_result','transport_audit'):
        p=run/(name+'.json')
        if p.exists():
            d=json.loads(p.read_text())
            metric[name]=({k:d.get(k) for k in ('diagnostic_pass','final_destination_error_mm','carry_max_object_angle_deg')}
                if name=='transport_audit' else d)
    metric['seating_hold_acks']=[e for e in summary['events'] if e['event']=='qp_seating_hold_ack']
    (run/'qp_metrics.json').write_text(json.dumps(metric,indent=2)+'\n')
    print(json.dumps(metric,indent=2))


if __name__=='__main__':analyze(Path(sys.argv[1]))
