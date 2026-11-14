#!/usr/bin/env python3
"""Installed entry point for the versioned C43 physical-contact reference."""
import argparse
import math
from datetime import datetime, timezone
import os
from pathlib import Path
import subprocess
from ament_index_python.packages import get_package_share_directory


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path)
    parser.add_argument('--gui',action='store_true')
    parser.add_argument('--external-gui',action='store_true',
        help='Run only the server; an independent GUI connects through IGN_PARTITION')
    parser.add_argument('--keep-open',action='store_true')
    parser.add_argument('--start-delay',type=float,default=0.)
    parser.add_argument('--prepare-only',action='store_true')
    parser.add_argument('--backend',choices=('jtc','qp-smoke','qp-carry','qp-executor'),default='jtc')
    parser.add_argument('--calibration-mass-kg',type=float,help='Explicit isolated stopping-calibration mass variant (0.25..0.60 kg)')
    parser.add_argument('--calibration-speed-scale',type=float,default=1.,help='Scale only QP transport speed for isolated stopping calibration (0.75..1.5)')
    parser.add_argument('--finite-stop-reserve-mm',type=float,default=0,help='Additional experimental stopping reserve (0..50 mm), calibration not approved')
    parser.add_argument('--enable-finite-guard',action='store_true',help='Enable bounded analytical object/environment check in QP update')
    parser.add_argument('--scene-precheck',action='store_true',help='Reject launch if full-scene discrete reference-path preflight fails')
    parser.add_argument('--enable-contact-safety',action='store_true',help='Experimental native contact whitelist and 2mm/0.5deg following guard')
    parser.add_argument('--enable-hold-resume',action='store_true',
        help='Enable guarded explicit operator HOLD/resume (qp-executor only)')
    parser.add_argument('--ros-domain-id',type=int,default=146,
        help='Isolated ROS domain for this experiment (default 146)')
    parser.add_argument('--inject-executor',choices=('missing-grasp','missing-support','retreat-abort',
        'destination-absent','carry-contact-loss','carry-contact-loss-regrip',
        'stale-contact','evidence-gap','executor-pause','hold-resume','unexpected-contact','right-wrist-load','finite-clearance'),
        help='Explicit fault injection or HOLD/resume test, only with qp-executor')
    parser.add_argument('--physics-step-ms',type=int,choices=(1,2,5),
        help='Physics step variant; default 2 ms for qp-executor, packaged 5 ms for other backends')
    args=parser.parse_args()
    if args.gui and args.external_gui:parser.error('--gui and --external-gui are mutually exclusive')
    if args.keep_open and not (args.gui or args.external_gui):parser.error('--keep-open requires --gui or --external-gui')
    if args.start_delay<0:parser.error('--start-delay must be nonnegative')
    if not 0<=args.ros_domain_id<=232:parser.error('--ros-domain-id must be between 0 and 232')
    if args.calibration_mass_kg is not None and (not math.isfinite(args.calibration_mass_kg) or not .25<=args.calibration_mass_kg<=.60):parser.error('calibration mass must be 0.25..0.60 kg')
    if not math.isfinite(args.calibration_speed_scale) or not .75<=args.calibration_speed_scale<=1.5:parser.error('calibration speed scale must be 0.75..1.5')
    if (args.calibration_mass_kg is not None or args.calibration_speed_scale!=1.) and args.inject_executor!='finite-clearance':parser.error('mass/speed calibration variants require isolated finite-clearance mode')
    if not math.isfinite(args.finite_stop_reserve_mm) or not 0<=args.finite_stop_reserve_mm<=50:parser.error('finite stopping reserve must be 0..50 mm')
    if args.finite_stop_reserve_mm and not args.enable_finite_guard:parser.error('stopping reserve requires --enable-finite-guard')
    if args.enable_finite_guard and args.backend!='qp-executor':parser.error('--enable-finite-guard requires qp-executor')
    if args.inject_executor=='finite-clearance' and not args.enable_finite_guard:parser.error('finite-clearance requires --enable-finite-guard')
    if args.scene_precheck and args.backend!='qp-executor':parser.error('--scene-precheck requires qp-executor')
    if args.enable_contact_safety and args.backend!='qp-executor':parser.error('--enable-contact-safety requires qp-executor')
    if args.enable_hold_resume and args.backend!='qp-executor':parser.error('--enable-hold-resume requires qp-executor')
    if args.inject_executor and args.backend!='qp-executor':parser.error('--inject-executor requires qp-executor')
    out=(args.out or Path.cwd()/'log'/('c43_contact_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%f'))).resolve()
    env=os.environ.copy()
    # This entry is a frozen reference; experimental overrides use the lab directly.
    for key in tuple(env):
        if key.startswith('C43_'):del env[key]
    env.update(C43_LAB_DUAL='1',C43_LAB_GUI=str(int(args.gui)),
        C43_EXTERNAL_GUI=str(int(args.external_gui)),
        C43_GUI_KEEP_OPEN=str(int(args.keep_open)),C43_GUI_START_DELAY_S=str(args.start_delay),
        C43_LAB_PREPARE_ONLY=str(int(args.prepare_only)),C43_ARM_BACKEND=args.backend)
    if args.calibration_mass_kg is not None:env['C43_CALIBRATION_MASS_KG']=str(args.calibration_mass_kg)
    if args.calibration_speed_scale!=1.:env['C43_CALIBRATION_SPEED_SCALE']=str(args.calibration_speed_scale)
    if args.finite_stop_reserve_mm:env['C43_FINITE_STOP_RESERVE_M']=str(args.finite_stop_reserve_mm/1000.)
    if args.enable_finite_guard:env['C43_ENABLE_FINITE_GUARD']='1'
    if args.scene_precheck:env['C43_SCENE_PREFLIGHT']='1'
    if args.enable_contact_safety:env['C43_ENABLE_CONTACT_SAFETY']='1'
    if args.enable_hold_resume:env['C43_ENABLE_HOLD_RESUME']='1'
    env['C43_ROS_DOMAIN_ID']=str(args.ros_domain_id)
    if args.physics_step_ms is not None:env['C43_PHYSICS_STEP_S']=str(args.physics_step_ms/1000.)
    elif args.backend=='qp-executor':env['C43_PHYSICS_STEP_S']='0.002'
    if args.inject_executor:env['C43_EXECUTOR_INJECT']=args.inject_executor
    share=Path(get_package_share_directory('gz_launch'))
    print('C43 packaged reference output:',out,flush=True)
    return subprocess.call(['python3',str(share/'scripts/c43_force_lab.py'),str(out),'dual_transport','.0249'],env=env)


if __name__=='__main__':raise SystemExit(main())
