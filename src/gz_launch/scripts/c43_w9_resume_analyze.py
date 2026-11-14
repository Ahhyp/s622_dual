#!/usr/bin/env python3
"""Audit explicit physical HOLD/resume; queued service replies are not motion ACKs."""
import argparse
import json
import math
import yaml
from pathlib import Path
from c43_dual_transport_analyze import audit


def analyze(run):
    audit(run)
    physical=json.loads((run/'transport_audit.json').read_text())
    formal=json.loads((run/'executor/summary.json').read_text())
    protocol=[json.loads(x) for x in (run/'executor/contact_protocol.jsonl').open()]
    summary=json.loads((run/'summary.json').read_text())
    rows=[json.loads(x) for x in (run/'qp_status.jsonl').open()]
    def event(name):return next((x for x in protocol if x['event']==name),None)
    held=event('operator_hold_applied');request=event('operator_resume_requested');resumed=event('operator_resume_applied')
    window=[r for r in rows if held and request and held['sim']+.03<=r['sim']<request['sim']]
    spans=[]
    if window and all(len(r['q_cmd'])==12 and all(math.isfinite(v) for v in r['q_cmd']) for r in window):
        spans=[max(r['q_cmd'][i] for r in window)-min(r['q_cmd'][i] for r in window) for i in range(12)]
    params=yaml.safe_load((run/'controllers.yaml').read_text())['dual_arm_qp_controller']['ros__parameters']
    dt=params.get('dt',.008);dq_bound=params['v_max']*dt
    first=next((r for r in rows if resumed and r['sim']>=resumed['sim'] and r['state']=='RUNNING'),None)
    replies=[e['detail'] for e in summary['events'] if e['event']=='w9_hold_service']
    checks=dict(
        physical_complete=physical['diagnostic_pass'] is True,
        formal_complete=formal['result']=='COMPLETE',
        early_resume_refused=any(r['stage']=='reject_early' and r['success'] is False for r in replies),
        explicit_hold_queued=any(r['stage']=='hold_request' and r['success'] is True for r in replies),
        explicit_resume_queued=any(r['stage']=='resume_request' and r['success'] is True for r in replies),
        applied_order=bool(held and request and resumed) and held['sim']<request['sim']<resumed['sim'],
        hold_observation_coverage=len(window)>=50 and window[-1]['sim']-window[0]['sim']>=.9,
        owned_hold=bool(window and held) and all(r['state']=='HOLD' and r['hold_origin']=='executor' and
            r['hold_request_seq']==held['seq'] and r['hold_request_result']==1 for r in window),
        both_arms_frozen=len(spans)==12 and max(spans)==0,
        trajectory_frozen=bool(window) and len({r['t_traj'] for r in window})==1,
        no_commits_while_held=bool(window) and all(not r['committed'] for r in window),
        running_ack=bool(first and held) and first['hold_request_seq']==held['seq'] and first['hold_origin']=='executor' and first['hold_request_result']==1,
        bounded_first_dq=bool(first) and len(first['dq'])==12 and all(math.isfinite(x) and abs(x)<=dq_bound+1e-12 for x in first['dq']),
        final_supported_clear=physical['last_native']['destination_support'] and not physical['last_native']['all_four'],
        cleanup=json.loads((run/'cleanup.json').read_text())['clean'] is True)
    result=dict(scope='explicit operator HOLD/resume in physical simulation; not full W9 safety acceptance',
        passed=all(checks.values()),checks=checks,hold_applied=held,resume_requested=request,resume_applied=resumed,
        hold_status_samples=len(window),left_q_cmd_span_rad=max(spans[:6]) if spans else None,
        right_q_cmd_span_rad=max(spans[6:]) if spans else None,first_resumed_dq=first['dq'] if first else None,
        dq_audit_bound_rad_per_update=dq_bound,dq_bound_note='configured v_max multiplied by controller dt; dq is increment per update',
        final_destination_error_mm=physical.get('final_destination_error_mm'),full_w9_acceptance=False)
    (run/'w9_resume_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return result

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path)
    raise SystemExit(not analyze(p.parse_args().run)['passed'])
