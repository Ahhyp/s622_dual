#!/usr/bin/env python3
"""Isolated existing-QP small/full physical carry; not formal W7 acceptance."""
import json
import os
import time
import numpy as np
import rclpy
from controller_manager_msgs.srv import SwitchController,ListControllers
from dual_arm_qp_controller.srv import StartTransport,HoldTransport
from dual_arm_qp_controller.msg import TransportStatus
from std_srvs.srv import Trigger
from c43_dual_transport import Transport
from c43_position_grip_probe import OUT,FINGERS
from c43_homing import LEFT_JOINTS,RIGHT_JOINTS


class QpSmoke(Transport):
    def __init__(self):
        super().__init__()
        self.status=None;self.status_wall=None;self.qp_rows=[]
        self.create_subscription(TransportStatus,'/dual_arm_qp_controller/transport_status',self.on_status,
            1 if os.environ.get('C43_ARM_BACKEND')=='qp-executor' else 100)
        self.switch_client=self.create_client(SwitchController,'/controller_manager/switch_controller')
        self.list_client=self.create_client(ListControllers,'/controller_manager/list_controllers')
        self.start_client=self.create_client(StartTransport,'/dual_arm_qp_controller/start_transport')
        self.reset_client=self.create_client(Trigger,'/dual_arm_qp_controller/reset_transport')
        self.hold_client=self.create_client(HoldTransport,'/dual_arm_qp_controller/hold_transport')

    def on_status(self,msg):
        self.status=msg;self.status_wall=time.monotonic()
        self.qp_rows.append(dict(sim=self.sim,mono=time.monotonic()-self.t0,state=msg.state,
            phase_name=msg.phase_name,t_traj=msg.t_traj,active=msg.active,committed=msg.committed,
            failure_code=msg.failure_code,protection=msg.protection,protection_complete=msg.protection_complete,
            hold_request_seq=msg.hold_request_seq,hold_request_result=msg.hold_request_result,
            hold_origin=msg.hold_origin,executor_heartbeat_required=msg.executor_heartbeat_required,
            executor_heartbeat_seen=msg.executor_heartbeat_seen,executor_heartbeat_age_s=msg.executor_heartbeat_age_s,
            q_state=list(msg.q_state),q_cmd=list(msg.q_cmd),dq=list(msg.dq),
            e_rel_pos_m=msg.e_rel_pos_m,e_rel_rot_rad=msg.e_rel_rot_rad,
            e_obj_pos_m=msg.e_obj_pos_m,e_obj_rot_rad=msg.e_obj_rot_rad,
            anchor_pos_gap_m=msg.anchor_pos_gap_m,anchor_rot_gap_rad=msg.anchor_rot_gap_rad,
            finite_geometry_guard_enabled=msg.finite_geometry_guard_enabled,finite_geometry_checked=msg.finite_geometry_checked,
            finite_geometry_violation=msg.finite_geometry_violation,finite_geometry_obstacle=msg.finite_geometry_obstacle,
            finite_geometry_native_fresh=msg.finite_geometry_native_fresh,finite_geometry_native_age_s=msg.finite_geometry_native_age_s,
            finite_geometry_source=msg.finite_geometry_source,finite_geometry_margin_m=msg.finite_geometry_margin_m,
            finite_geometry_check_us=msg.finite_geometry_check_us,qp_solve_time_ms=msg.qp_solve_time_ms))

    def service(self,client,request):
        if not client.wait_for_service(timeout_sec=30):raise RuntimeError('service unavailable: '+client.srv_name)
        return self.wait_future(client.call_async(request),30)

    def switch(self,to_qp):
        req=SwitchController.Request();req.strictness=SwitchController.Request.STRICT
        arms=['left_arm_controller','right_arm_controller'];qp=['dual_arm_qp_controller']
        req.deactivate_controllers=arms if to_qp else qp
        req.activate_controllers=qp if to_qp else arms
        result=self.service(self.switch_client,req)
        if not result.ok:raise RuntimeError('controller switch refused')
        controllers=self.service(self.list_client,ListControllers.Request()).controller
        records={c.name:dict(state=c.state,claimed_interfaces=list(c.claimed_interfaces)) for c in controllers}
        active={c.name for c in controllers if c.state=='active'}
        if to_qp:
            if not set(qp)<=active or set(arms)&active:raise RuntimeError('arm command ownership is not exclusive to QP')
            claimed=records[qp[0]]['claimed_interfaces']
            if set(claimed)!={j+'/position' for j in LEFT_JOINTS+RIGHT_JOINTS}:
                raise RuntimeError('QP did not claim the 12 expected position interfaces')
        elif not set(arms)<=active or set(qp)&active:raise RuntimeError('JTC ownership restore failed')
        if {'left_hand_controller','right_hand_controller'}&active:raise RuntimeError('hand JTC conflicts with native drives')
        (OUT/('ownership_qp.json' if to_qp else 'ownership_jtc_restored.json')).write_text(json.dumps(records,indent=2)+'\n')
        self.event('controller_ownership_verified',backend='qp' if to_qp else 'jtc',controllers=records)

    def carry(self,path):
        full=os.environ.get('C43_ARM_BACKEND')=='qp-carry'
        qp_task=json.loads((OUT/'qp_task.json').read_text())
        self.switch(True)
        self.phase='settle';self.spin(1);self.snapshot('qp_zero_hold_end')
        if self.status is None:raise RuntimeError('no QP status after activation')
        sample0=np.array(self.pose('c43_sample')[:3]);request=StartTransport.Request()
        request.phase_name=['lift','transport','lower']
        request.phase_duration=[4.,3.,4.]
        request.phase_dx=[0.,0.,0.];request.phase_dy=[0.,0.,0.];request.phase_dz=[.01,0.,-.01]
        if full:
            overtravel=qp_task['seat_overtravel_m']
            if not 0<overtravel<=.0005:raise RuntimeError('seating overtravel outside diagnostic 0.5 mm bound')
            request.phase_name=['lift','hold','transport','lower','seat']
            request.phase_duration=[6.,5.,8.,6.,qp_task['seat_trajectory_duration_s']]
            request.phase_dx=[0.,0.,0.,0.,0.];request.phase_dy=[0.,0.,.12,0.,0.];request.phase_dz=[.06,0.,0.,-.06,-overtravel]
        request.dt=.008;request.time_scaling='smoothstep'
        accepted=self.service(self.start_client,request)
        self.event('qp_start_response',accepted=accepted.accepted,message=accepted.message)
        if not accepted.accepted:raise RuntimeError('QP start refused: '+accepted.message)
        began=self.sim;running=False;last_phase=None;hold_requested=False;seat_held=False
        self.event('qp_reference_spec',phases=list(request.phase_name),durations=list(request.phase_duration),
            dx=list(request.phase_dx),dy=list(request.phase_dy),dz=list(request.phase_dz))
        while self.sim-began<sum(request.phase_duration)+15:
            self.spin(.02)
            if self.status_wall is None or time.monotonic()-self.status_wall>2:raise RuntimeError('QP status stale')
            msg=self.status
            if msg.state=='HOLD' and hold_requested and msg.hold_request_seq==qp_task['seat_contact_hold_seq'] and msg.hold_request_result==1:
                seat_held=True;self.event('qp_seating_hold_ack',seq=msg.hold_request_seq,t_traj=msg.t_traj);break
            if msg.state in ('HOLD','FAULT','ABORT'):raise RuntimeError('QP stopped: '+msg.state+'/'+msg.failure_code)
            running=running or msg.active or msg.state=='RUNNING'
            if msg.phase_name!=last_phase and msg.phase_name in request.phase_name:
                if last_phase:
                    self.snapshot({'lift':'lift_end','hold':'hold_lift_end','transport':'transfer_end','lower':'place_end','seat':'seat_end'}[last_phase])
                last_phase=msg.phase_name
                self.phase={'lift':'lift','hold':'hold_lift','transport':'transfer' if full else 'hold_lift','lower':'place','seat':'place'}[msg.phase_name]
            if full and msg.phase_name=='seat' and not hold_requested:
                with self.lock:
                    recent=self.contacts[-5:]
                    supported=len(recent)==5 and all('c43_destination' in json.dumps(c.get('sample_collision_pairs',[])) for c in recent)
                if supported:
                    hold_requested=True;hold=HoldTransport.Request();hold.seq=qp_task['seat_contact_hold_seq']
                    hold.reason='diagnostic destination support observed; freeze bounded seating'
                    answer=self.service(self.hold_client,hold)
                    if not answer.accepted:raise RuntimeError('seating HOLD request refused: '+answer.message)
                    self.event('qp_seating_hold_requested',seq=hold.seq)
            if running and msg.state=='COMPLETE':break
        else:raise RuntimeError('QP cycle did not complete within simulation deadline')
        if full and not seat_held:
            raise RuntimeError('bounded seating ended without matching support-contact HOLD/ACK')
        self.phase='place';self.spin(2);self.snapshot('qp_cycle_end')
        carrying=[r for r in self.rows if r['phase'] in ('lift','hold_lift','transfer','place') and r['sample']]
        peak=max(r['sample'][2] for r in carrying)-sample0[2]
        if peak<(.055 if full else .008):raise RuntimeError('QP reported complete but physical lift too small')
        expected=np.array(path['destination_xyz']) if full else sample0
        if np.linalg.norm(np.array(self.pose('c43_sample')[:3])-expected)>(.01 if full else .005):
            raise RuntimeError('QP cycle ended outside the required physical target tolerance')
        reset=self.service(self.reset_client,Trigger.Request())
        if not reset.success:raise RuntimeError('QP reset refused: '+reset.message)
        self.spin(.2);self.switch(False)
        if full:
            tail=dict(path,stages=[s for s in path['stages'] if s['name'] in
                ('retract','return_over_source','return_pre','home_left','home_right')])
            super().carry(tail)
            result=dict(physical_lift_peak_mm=peak*1000,final_sample=self.pose('c43_sample'),
                qp_status_rows=len(self.qp_rows),formal_w7_complete=False,
                contact_seating_hold_acknowledged=seat_held,
                scope='existing QP physical carry with local seating HOLD/ACK; formal executor integration not performed')
            (OUT/'qp_carry_result.json').write_text(json.dumps(result,indent=2)+'\n')
            self.event('qp_contact_carry_complete',**result);return
        self.carry_offsets=None;self.phase='release';self.clamp(-1);self.spin(2)
        approach=json.loads((OUT/'approach.json').read_text())
        self.phase='retreat'
        self.trajectory('right_arm_controller',RIGHT_JOINTS,list(reversed(approach['right_descent']))[1:],6)
        self.trajectory('left_arm_controller',LEFT_JOINTS,list(reversed(approach['left_descent']))[1:],6)
        home=[0.,-1.5708,0.,0.,0.,0.]
        self.trajectory('left_arm_controller',LEFT_JOINTS,[home],8)
        self.trajectory('right_arm_controller',RIGHT_JOINTS,[home],8)
        self.phase='final';self.spin(5);self.snapshot('qp_smoke_final')
        with self.lock:
            if any(c['robot_links'] for c in self.contacts[-60:]):raise RuntimeError('robot contact persists after QP smoke retreat')
            if not any('c43_support' in json.dumps(c.get('sample_collision_pairs',[])) for c in self.contacts[-40:]):
                raise RuntimeError('sample not on source support after smoke')
        result=dict(physical_lift_peak_mm=peak*1000,final_sample=self.pose('c43_sample'),
            qp_status_rows=len(self.qp_rows),formal_w7_complete=False,
            scope='controller admission and small lift/return; no executor integration or full 120 mm QP carry')
        (OUT/'qp_smoke_result.json').write_text(json.dumps(result,indent=2)+'\n')
        self.event('qp_contact_smoke_complete',**result)

    def finish(self,error):
        super().finish(error)
        with (OUT/'qp_status.jsonl').open('w') as f:
            for r in self.qp_rows:f.write(json.dumps(r)+'\n')


def main():
    rclpy.init();node=QpSmoke();error=None
    try:node.run()
    except Exception as exc:error=repr(exc);node.event('error',detail=error)
    finally:
        node.finish(error);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return int(error is not None)


if __name__=='__main__':raise SystemExit(main())
