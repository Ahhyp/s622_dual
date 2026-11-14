#!/usr/bin/env python3
"""Audit physical W9 stopping evidence; never equates task failure with safe stop."""
import argparse
import csv
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def analyze(run):
    conditions=json.loads((run/'conditions.json').read_text())
    mode=conditions.get('C43_EXECUTOR_INJECT','')
    formal=json.loads((run/'executor/summary.json').read_text())
    physical=json.loads((run/'summary.json').read_text())
    events=list(csv.DictReader((run/'executor/events.csv').open()))
    protocol=[];protocol_warnings=[]
    lines=(run/'executor/contact_protocol.jsonl').read_text().splitlines()
    for i,line in enumerate(lines):
        try:protocol.append(json.loads(line))
        except json.JSONDecodeError:
            # Old adapter kept appending optional evidence AFTER its terminal
            # flush. Preserve and disclose a torn teardown tail; never ignore
            # malformed data before the authoritative stop record.
            if i==len(lines)-1 and any(x['event']=='terminal_stop' for x in protocol):
                protocol_warnings.append('truncated evidence tail after terminal_stop; raw file preserved')
            else:raise
    statuses=[json.loads(x) for x in (run/'qp_status.jsonl').open()]
    capture=json.loads((run/'w9_fault_capture.json').read_text())
    commands=[x for x in protocol if x['event']=='command']
    protections=[x for x in events if x['kind']=='protection']
    stop=next((x for x in protocol if x['event']=='terminal_stop'),{})
    trigger=capture.get('trigger')
    code=protections[0]['code'] if protections else ''
    first_abort=next((i for i,x in enumerate(capture['status_rows']) if x['state']=='ABORT'),None)
    stopped=[] if first_abort is None else capture['status_rows'][first_abort:]
    spans=[]
    if stopped and all(len(x['q_cmd'])==12 and all(math.isfinite(v) for v in x['q_cmd']) for x in stopped):
        spans=[max(x['q_cmd'][i] for x in stopped)-min(x['q_cmd'][i] for x in stopped) for i in range(12)]
    window=stopped[-1]['sim']-stopped[0]['sim'] if len(stopped)>1 else 0
    expected={
        'destination-absent':('release_failed',),
        'carry-contact-loss':('grasp_check_failed',),
        'carry-contact-loss-regrip':('grasp_check_failed',),
        'stale-contact':('stale_state',),
        'evidence-gap':('stale_state',),
        'executor-pause':('stale_state','executor_lost'),
        'unexpected-contact':('object_env_contact',),
        'right-wrist-load':('follower_divergence',),
        'finite-clearance':('object_table_clearance',)}
    checks=dict(
        supported_case=mode in expected,
        fault_cause_matches=code in expected.get(mode,()),
        task_did_not_complete=formal['result'] in ('HOLD','FAULT','ABORT') and physical['error'] is not None,
        no_release_or_retreat=not any(x['phase'] in ('RELEASE','RETREAT') for x in commands),
        no_physical_release=not any(x['event'] in ('released','physical_transport_complete') for x in physical['events']),
        stop_command=bool(commands) and commands[-1]['phase']=='STOP',
        plant_stop_ack=stop.get('plant_ack') is True,
        actual_controller_abort=stop.get('controller_stop_confirmed') is True and
            stop.get('controller_state')=='ABORT' and stop.get('controller_stop_outcome')=='confirmed_by_controller',
        stopped_sample_window=window>=.9,  # data coverage of the planned 1 s observation
        no_restart_in_observation=bool(stopped) and all(x['state']=='ABORT' and not x['committed'] for x in stopped),
        both_arm_commands_frozen=len(spans)==12 and max(spans)<=1e-12,  # serialization/equality tolerance
        unsafe_reset_refused=capture['recovery_probes'].get('reset',{}).get('success') is False,
        unsafe_start_refused=capture['recovery_probes'].get('start',{}).get('success') is False,
        non_gui=conditions.get('C43_LAB_GUI')=='0')
    # Native contacts are independent of the evidence masks.
    raw=[json.loads(x) for x in (run/'contacts.jsonl').open()]
    if mode=='destination-absent':
        world=ET.parse(run/'world.sdf').getroot().find('world')
        checks['destination_really_absent']=not any(x.findtext('name')=='c43_destination' for x in world.findall('include'))
        checks['no_raw_destination_contact']=not any('c43_destination' in json.dumps(x.get('sample_collision_pairs',[])) for x in raw)
        checks['lower_attempted']=any(x['to']=='LOWER' for x in events if x['kind']=='advance')
    else:
        checks['injection_triggered_after_real_grasp']=bool(trigger) and trigger.get('raw_four') is True and trigger.get('phase')=='settle'
    if mode in ('carry-contact-loss','carry-contact-loss-regrip') and trigger:
        cutoff=capture['observe_end_sim']
        checks['native_contact_really_lost']=any(trigger['sim']<=x['sim']<=cutoff and len(x['fingers'])<4 for x in raw)
        checks['physical_left_open_command']=any(x['event']=='joint_drives_N' and x['detail'].get('values')==[2,-2,-5,5] for x in physical['events'])
    if mode=='carry-contact-loss-regrip' and trigger:
        end=capture['sample'];start=trigger['sample']
        checks['original_grip_reasserted']=any(x['event']=='grip_hold_reasserted' for x in physical['events'])
        tail=[x for x in raw if capture['observe_end_sim']-.2<=x['sim']<=capture['observe_end_sim']]
        checks['native_four_contacts_reacquired']=bool(tail) and all(len(x['fingers'])==4 for x in tail)
        checks['payload_translation_within_resume_bound']=math.dist(start[:3],end[:3])<=.005
        dot=abs(sum(a*b for a,b in zip(start[3:],end[3:])))
        checks['payload_rotation_within_resume_bound']=2*math.acos(min(1,dot))<=math.radians(2)
    if mode=='stale-contact' and trigger:
        checks['old_contact_timestamp_seen']=any(x['event']=='evidence' and x['sim']>=trigger['sim'] and x['sim']-x['contact_stamp']>.1 for x in protocol)
    if mode=='evidence-gap' and trigger:
        evidence=[x for x in protocol if x['event']=='evidence']
        checks['evidence_gap_observed']=any(b['sim']-a['sim']>=.09 and a['sim']<=trigger['sim']+.03<=b['sim'] for a,b in zip(evidence,evidence[1:]))
    if mode=='finite-clearance':
        detected=[r for r in statuses if r.get('finite_geometry_violation')]
        first_native=next((r['sim'] for r in raw if 'c43_w9_obstacle' in json.dumps(r['sample_collision_pairs'])),None)
        checks['analytical_guard_enabled']=bool(detected) and all(r.get('finite_geometry_guard_enabled') and r.get('finite_geometry_checked') for r in detected)
        checks['analytical_obstacle_identified']=bool(detected) and detected[0].get('finite_geometry_obstacle')=='c43_w9_obstacle::base_link::collision'
        checks['analytical_margin_violated']=bool(detected) and detected[0].get('finite_geometry_margin_m',1)<=1e-12
        checks['guard_no_later_than_first_native_contact']=bool(detected) and (first_native is None or detected[0]['sim']<=first_native+.002)
    if mode in ('unexpected-contact','right-wrist-load'):
        violation=next((e for e in physical['events'] if e['event']=='physical_safety_violation'),None)
        expected_code='object_env_contact' if mode=='unexpected-contact' else 'follower_divergence'
        checks['native_safety_witness_recorded']=bool(violation) and violation['detail']['code']==expected_code
        checks['safety_abort_received']=any(e['event']=='physical_safety_abort' and e['code']==expected_code for e in protocol)
        if mode=='unexpected-contact':
            checks['real_obstacle_present']=ET.parse(run/'world.sdf').getroot().find("world/model[@name='c43_w9_obstacle']") is not None
            checks['real_obstacle_contact']=any('c43_w9_obstacle' in json.dumps(r['sample_collision_pairs']) for r in raw)
        else:
            checks['external_right_wrist_load_issued']=any(e['event']=='w9_right_wrist_load_command' and e['detail']['apply'] is True for e in physical['events'])
            checks['external_load_clear_issued']=any(e['event']=='w9_right_wrist_load_command' and e['detail']['apply'] is False for e in physical['events'])
            checks['actual_follow_bound_exceeded']=any(r['anchor_pos_gap_m']>.002 or r['anchor_rot_gap_rad']>math.radians(.5) for r in statuses)
    if mode=='executor-pause':
        paused=next((x for x in physical['events'] if x['event']=='w9_executor_paused'),None)
        resumed=next((x for x in physical['events'] if x['event']=='w9_executor_continued'),None)
        held=[x for x in statuses if x['state']=='HOLD' and x['failure_code']=='executor_lost']
        checks['actual_executor_pause_and_continue']=bool(paused and resumed)
        checks['controller_watchdog_detected_loss']=bool(held)
        checks['watchdog_fired_while_executor_paused']=bool(paused and resumed) and any(paused['mono']<=x['mono']<=resumed['mono'] for x in held)
        checks['trajectory_frozen_by_watchdog']=bool(held) and max(x['t_traj'] for x in held)==min(x['t_traj'] for x in held)
    else:held=[]
    physical_fault_stamp=None
    if mode=='unexpected-contact':
        physical_fault_stamp=next((r['sim'] for r in raw if 'c43_w9_obstacle' in json.dumps(r['sample_collision_pairs'])),None)
    elif mode=='finite-clearance':
        physical_fault_stamp=next((r['sim'] for r in statuses if r.get('finite_geometry_violation')),None)
    elif mode=='right-wrist-load':
        physical_fault_stamp=next((r['sim'] for r in statuses if r['anchor_pos_gap_m']>.002 or r['anchor_rot_gap_rad']>math.radians(.5)),None)
    result=dict(scope='W9 physical-profile fault stopping; not full W9 acceptance',mode=mode,
        checks=checks,passed=all(checks.values()),executor_terminal=formal['result'],first_failure_code=code,
        fault_latency_sim_s=float(protections[0]['t'])-trigger['sim'] if protections and trigger else None,
        physical_fault_to_executor_abort_sim_s=float(protections[0]['t'])-physical_fault_stamp if mode!='finite-clearance' and protections and physical_fault_stamp is not None else None,
        timing_note='finite guard/status observer and executor use different receipt stamps; no reaction-latency claim' if mode=='finite-clearance' else None,
        controller_watchdog_latency_sim_s=held[0]['sim']-trigger['sim'] if held and trigger else None,
        controller_stop=stop,post_stop_status_samples=len(stopped),post_stop_window_sim_s=window,
        left_command_span_rad=max(spans[:6]) if spans else None,right_command_span_rad=max(spans[6:]) if spans else None,
        sample_after_stop=capture.get('sample'),protocol_warnings=protocol_warnings,
        payload_tilt_after_stop_deg=2*math.degrees(math.acos(min(1,abs(capture['sample'][6])))) if capture.get('sample') else None,
        positive_online_recovery_validated=False,full_w9_acceptance=False)
    (run/'w9_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run',type=Path)
    raise SystemExit(not analyze(parser.parse_args().run)['passed'])
