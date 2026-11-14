#!/usr/bin/env python3
"""Approach-only dual-arm candidate validation, grippers stay open; no weld."""
import json
import os
import rclpy
from c43_position_grip_probe import Probe,OUT,FINGERS
from c43_homing import LEFT_JOINTS,RIGHT_JOINTS


class Dry(Probe):
    def on_js(self,msg):
        super().on_js(msg)
        if self.phase=='homing' and any(c.get('robot_links',c['fingers']) for c in self.contacts[-20:]):
            raise RuntimeError('unexpected robot/sample contact during candidate approach')

    def run(self):
        path=json.loads(open(os.environ['C43_CANDIDATE_JSON']).read())
        if not path['path_collision_free']:raise RuntimeError('candidate did not pass whole-path FCL')
        for name,client in self.action_clients.items():
            if not client.wait_for_server(timeout_sec=45):raise RuntimeError(name+' unavailable')
        self.spin(1)
        self.trajectory('left_hand_controller',FINGERS[:2],[[.0305,-.0305]],1.5)
        self.trajectory('right_hand_controller',FINGERS[2:],[[.0305,-.0305]],1.5)
        self.phase='homing'
        self.trajectory('right_arm_controller',RIGHT_JOINTS,[path['right_pre']],8)
        self.trajectory('left_arm_controller',LEFT_JOINTS,[path['left_pre']],8)
        self.trajectory('left_arm_controller',LEFT_JOINTS,path['left_descent'],6)
        self.trajectory('right_arm_controller',RIGHT_JOINTS,path['right_descent'],6)
        self.phase='candidate_hold';self.spin(5)
        errors={side:max(abs(self.q[j]-target) for j,target in zip(joints,path[side+'_descent'][-1]))
            for side,joints in [('left',LEFT_JOINTS),('right',RIGHT_JOINTS)]}
        self.event('candidate_achieved',max_joint_errors_rad=errors)
        if max(errors.values())>.01:raise RuntimeError('achieved candidate joint residual too large')
        (OUT/'candidate_achieved.json').write_text(json.dumps(dict(errors=errors,joints=self.q,
            poses={name:self.pose(name) for name in ['c43_sample','left_wrist3_link','right_wrist3_link']},target=path),indent=2))


def main():
    rclpy.init();node=Dry();error=None
    try:node.run()
    except Exception as exc:error=repr(exc);node.event('error',message=error)
    finally:
        node.save(error);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return int(error is not None)

if __name__=='__main__':raise SystemExit(main())
