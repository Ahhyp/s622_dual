#!/usr/bin/env python3
"""Run a small, isolated stopping pilot inside the sourced ROS container.

Expected task ABORT is assessed by the stopping audits, not the launch exit code.
Existing outputs are refused; raw failed attempts remain available for review.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import subprocess
import sys

CASES = [('nominal_r1', .35, 1.), ('nominal_r2', .35, 1.),
         ('nominal_r3', .35, 1.), ('speed125_r1', .35, 1.25),
         ('mass050_r1', .50, 1.)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('out', type=Path)
    parser.add_argument('--domain-start', type=int, default=170)
    args = parser.parse_args()
    if not 0 <= args.domain_start <= 228:
        parser.error('five ROS domains must fit within 0..232')
    root = args.out.resolve()
    root.mkdir(parents=True, exist_ok=True)
    if any((root / name).exists() for name, _, _ in CASES):
        parser.error('case output already exists; choose a fresh batch directory')
    scripts = Path(__file__).resolve().parent

    def run(item):
        i, (name, mass, speed) = item
        out = root / name
        cmd = ['ros2', 'run', 'gz_launch', 'c43_run_contact_transport.py',
               '--backend', 'qp-executor', '--enable-finite-guard',
               '--finite-stop-reserve-mm', '8', '--inject-executor', 'finite-clearance',
               '--calibration-mass-kg', str(mass), '--calibration-speed-scale', str(speed),
               '--ros-domain-id', str(args.domain_start + i), '--out', str(out)]
        record = dict(name=name, mass_kg=mass, transport_speed_scale=speed, command=cmd)
        print('START', name, flush=True)
        with (root / (name + '.launch.log')).open('w') as log:
            record['launch_exit'] = subprocess.call(cmd, stdout=log, stderr=subprocess.STDOUT)
            record['audits'] = {}
            for script in ('c43_w9_analyze.py', 'c43_finite_runtime_analyze.py'):
                record['audits'][script] = subprocess.call(
                    [sys.executable, str(scripts / script), str(out)], stdout=log, stderr=subprocess.STDOUT)
        record['passed'] = all(code == 0 for code in record['audits'].values())
        (root / (name + '.result.json')).write_text(json.dumps(record, indent=2) + '\n')
        print('DONE', name, record['passed'], flush=True)
        return record

    with ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(run, enumerate(CASES)))
    summary = dict(scope='five-case simulation pilot; no hardware or statistical safety certification',
                   stopping_reserve_calibration_approved=False,
                   passed=all(r['passed'] for r in results), cases=results)
    (root / 'batch.json').write_text(json.dumps(summary, indent=2) + '\n')
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
