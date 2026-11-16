#!/usr/bin/env python3
"""Offline real-geometry CBF experiments; does not start Gazebo."""
import argparse
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('out', type=Path)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    from ament_index_python.packages import get_package_share_directory
    profile = Path(get_package_share_directory('gz_launch')) / 'config/c43_contact_transport_v1'
    transport = json.loads((profile / 'transport.json').read_text())
    srdf = Path(get_package_share_directory('s622_moveit_config')) / 'config/s622_dual_arm.srdf'
    lift = next(s for s in transport['stages'] if s['name'] == 'lift')['q12'][-1]
    cases = [('clear_off', [0, .30, .335], [0, 0, 0], 'off', 0),
             ('blocked_off', [0, .06, .335], [0, 0, 0], 'off', 0),
             ('blocked_on', [0, .06, .335], [0, 0, 0], 'on', 0),
             ('approaching_on', [0, .10, .335], [0, -.005, 0], 'on', 0),
             ('raised_path_on', [0, .06, .335], [0, 0, 0], 'on', 1)]
    if any((args.out / name).exists() for name, *_ in cases):
        parser.error('outputs exist; choose a new directory')
    results = []
    for name, center, velocity, mode, raised in cases:
        out = args.out / name
        out.mkdir()
        values = [transport['grasp_q12'], lift, [0, 0, .275], center,
                  velocity, [0, .12, 0], [.015, .010, raised]]
        input_path = out / 'input.txt'
        input_path.write_text('\n'.join(' '.join(map(str, row)) for row in values) + '\n')
        cmd = ['ros2', 'run', 'dual_arm_pose_optimization', 'c3_obstacle_probe',
               str(profile / 'robot.urdf'), str(srdf), str(input_path), str(out), mode, '1000']
        with (out / 'run.log').open('w') as log:
            code = subprocess.call(cmd, stdout=log, stderr=subprocess.STDOUT)
        result = dict(name=name, command=cmd, exit_code=code)
        if (out / 'summary.json').exists():
            result.update(json.loads((out / 'summary.json').read_text()))
        results.append(result)
        print(name, code, result.get('outcome'), flush=True)
    (args.out / 'batch.json').write_text(json.dumps(results, indent=2) + '\n')
    checks = {}
    by_name = {r['name']: r for r in results}
    for name in ('clear_off', 'raised_path_on'):
        r = by_name[name]
        checks[name + '_goal'] = r.get('goal_reached_within_5mm') is True and r.get('qp_failures') == 0
    r = by_name['blocked_off']
    checks['off_predicts_collision'] = r.get('outcome') == 'collision_candidate'
    for name in ('blocked_on', 'approaching_on'):
        r = by_name[name]
        checks[name + '_stays_clear'] = (r.get('outcome') == 'rollout_completed'
            and r.get('collisions') == 0 and r.get('min_distance_m', 0) >= .0099
            and r.get('goal_reached_within_5mm') is False and r.get('qp_failures') == 0)
    for name in ('blocked_on', 'approaching_on', 'raised_path_on'):
        r = by_name[name]
        checks[name + '_constraints'] = (r.get('gradient_checked') is True
            and r.get('initial_gradient_step_check_error', 1) < 1e-5
            and r.get('max_row_residual', 1) <= 1e-6
            and r.get('max_relative_position_m', 1) < .002
            and r.get('max_relative_angle_rad', 1) < .01)
    if by_name['raised_path_on'].get('goal_reached_within_5mm'):
        # Export the packaged static world independently of the QP experiment.
        # Add the same destination as the physical launcher. A circumscribed
        # box makes the independent sphere-obstacle check conservative.
        from c43_scene_precheck import export_scene
        full = args.out / 'full_scene'
        full.mkdir()
        source = full / 'source'
        source.mkdir()
        world = ET.parse(profile / 'world.sdf')
        w = world.getroot().find('world')
        include = ET.SubElement(w, 'include')
        ET.SubElement(include, 'uri').text = 'model://c43_support'
        ET.SubElement(include, 'name').text = 'c43_destination'
        ET.SubElement(include, 'pose').text = '0 .12 0 0 0 0'
        model = ET.SubElement(w, 'model', name='c3_obstacle')
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = '0 .06 .335 0 0 0'
        collision = ET.SubElement(ET.SubElement(model, 'link', name='link'), 'collision', name='collision')
        ET.SubElement(ET.SubElement(ET.SubElement(collision, 'geometry'), 'box'), 'size').text = '.03 .03 .03'
        world.write(source / 'world.sdf')
        export_scene(source, full)
        cmd = ['ros2', 'run', 'dual_arm_pose_optimization', 'c43_scene_path_check',
               str(profile / 'robot.urdf'), str(srdf), str(full / 'scene.txt'),
               str(args.out / 'raised_path_on/states.txt'), str(full)]
        with (full / 'run.log').open('w') as log:
            code = subprocess.call(cmd, stdout=log, stderr=subprocess.STDOUT)
        checks['independent_full_scene'] = code == 0 and json.loads((full / 'summary.json').read_text())['passed']
    else:
        checks['independent_full_scene'] = False
    audit = dict(scope='offline geometry and QP feasibility; no Gazebo',
                 passed=all(checks.values()), checks=checks)
    (args.out / 'audit.json').write_text(json.dumps(audit, indent=2) + '\n')
    print('audit passed:', audit['passed'], flush=True)
    return 0 if audit['passed'] else 1

if __name__ == '__main__':
    raise SystemExit(main())
