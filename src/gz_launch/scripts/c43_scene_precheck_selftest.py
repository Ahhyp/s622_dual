#!/usr/bin/env python3
"""Counterexamples for the whole-scene preflight (offline, never physical runs)."""
import argparse,json,subprocess
from pathlib import Path
from ament_index_python.packages import get_package_prefix


def main():
    p=argparse.ArgumentParser();p.add_argument('baseline',type=Path);p.add_argument('out',type=Path);a=p.parse_args();a.out.mkdir(exist_ok=False)
    scene=(a.baseline/'scene.txt').read_text();states=(a.baseline/'states.txt').read_text().splitlines()
    home=states[0];grasp=next(x for x in states if x.split()[1]=='GRASP')
    binary=Path(get_package_prefix('dual_arm_pose_optimization'))/'lib/dual_arm_pose_optimization/c43_scene_path_check'
    results={}
    def object_at(row,xyz,phase=None):
        columns=row.split();columns[18:21]=list(map(str,xyz))
        if phase:columns[1]=phase
        return ' '.join(columns)
    def run(name,state,world=scene,expected=1,pair=None):
        out=a.out/name;out.mkdir();(out/'scene.txt').write_text(world);(out/'states.txt').write_text(state+'\n')
        with (out/'checker.log').open('w') as log:rc=subprocess.call([str(binary),str(a.baseline/'robot.urdf'),str(a.baseline/'robot.srdf'),str(out/'scene.txt'),str(out/'states.txt'),str(out)],stdout=log,stderr=subprocess.STDOUT)
        rows=[json.loads(x) for x in (out/'states_audit.jsonl').read_text().splitlines()] if (out/'states_audit.jsonl').exists() else []
        has_pair=pair is None or any(any(pair in value for value in row['pairs']) for row in rows)
        results[name]=dict(returncode=rc,expected=expected,expected_pair_seen=has_pair,passed=rc==expected and has_pair)
    run('valid_scene_and_supported_start',home,expected=0)
    run('sample_table_penetration',object_at(home,[.4,0,-.01]),pair='c43_table::link::surface')
    first_leg=next(line.split() for line in scene.splitlines() if 'front_left_leg' in line)
    run('sample_table_leg_penetration',object_at(home,[float(x) for x in first_leg[5:8]]),pair='c43_table::link::front_left_leg')
    run('support_side_not_top',object_at(home,[.045,0,.125]),pair='c43_support::base_link::collision')
    run('destination_contact_before_lower',object_at(grasp,[0,.12,.2748],'TRANSPORT'),pair='c43_destination::base_link::collision')
    run('wrist_sample_not_exempt',object_at(grasp,[-.0811,0,.50942]),pair='right_wrist')
    extra='w9_robot_obstacle box .12 .05 .05 -.0811 0 .50942 0 0 0 1\n'
    run('robot_unknown_obstacle_not_exempt',grasp,scene+extra,pair='w9_robot_obstacle')
    moved=[]
    for line in scene.splitlines():
        if 'front_left_leg' in line:
            c=line.split();c[5:8]=['-.0811','0','.50942'];line=' '.join(c)
        moved.append(line)
    run('robot_table_leg_not_exempt',grasp,'\n'.join(moved)+'\n',pair='c43_table::link::front_left_leg')
    run('missing_world_geometry_fails_closed',home,'\n'.join(x for x in scene.splitlines() if 'back_left_leg' not in x)+'\n',expected=2)
    cols=home.split();cols[1]='UNKNOWN';run('unknown_phase_fails_closed',' '.join(cols),expected=2)
    cols=home.split();cols[21:25]=['0','0','0','0'];run('invalid_sample_quaternion_fails_closed',' '.join(cols),expected=2)
    result=dict(kind='offline full-scene preflight counterexamples',tests=len(results),checks=results,passed=all(x['passed'] for x in results.values()))
    (a.out/'results.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return int(not result['passed'])
if __name__=='__main__':raise SystemExit(main())
