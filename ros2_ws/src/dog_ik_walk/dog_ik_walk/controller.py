"""Finite four-beat IK joint trajectory; no IMU, body estimator or physics stepping."""
import threading
import json
from pathlib import Path
from collections import deque
import numpy as np
from dog_clock import now
from dog_link import Cancelled,GLOBAL_ENABLE
from dog_protocol import encode_model_mit,encode_mit
from dog_gait import DogGait
from dog_runtime import minimum_jerk,calibration_problems
from tools.dog_slow_stand_test import StandTestController


def feedback_to_model(rows,calibration):
    by_id={r['id']:r for r in rows}
    return tuple(np.array([c['direction']*(by_id[c['can_id']][key]-(c['zero_rad'] if key=='position' else 0)) for c in calibration]) for key in ('position','velocity','torque'))


def align_encoder_branches(rows,calibration,limits):
    """Select the calibrated zero on the nearest output-encoder revolution.

    Only run with fresh post-enable feedback and zero commanded gains. Never
    wrap an active trajectory or alter the measured zero/direction on disk.
    """
    by_id={r['id']:r for r in rows};aligned=[];events=[]
    for i,row in enumerate(calibration):
        current=dict(row);position=by_id[row['can_id']]['position'];direction=row['direction']
        # Keep the original calibration immutable across repeated enable/zero cycles.
        zero=row.get('original_zero_rad',row['zero_rad']);current['original_zero_rad']=zero
        if not np.isfinite([position,zero,direction]).all() or direction not in (-1,1):
            raise ValueError('Invalid encoder position/calibration')
        turns=int(round((position-zero)/(2*np.pi)));new_zero=zero+turns*2*np.pi
        q=direction*(position-new_zero);current['zero_rad']=new_zero;aligned.append(current)
        events.append(dict(can_id=row['can_id'],position_rad=position,previous_zero_rad=row['zero_rad'],original_zero_rad=zero,
                           session_zero_rad=new_zero,branch_shift_turns=turns,model_position_rad=q))
    return aligned,events


class WriteGate:
    def __init__(self,link,allowed):self.link=link;self.allowed=allowed;self.writes=0
    def __getattr__(self,name):return getattr(self.link,name)
    def send(self,packet,cancelled=None):
        if not self.allowed:raise RuntimeError('Monitor-only deployment; motor writes disabled')
        self.link.send(packet,cancelled);self.writes+=1


class IkWalkController(StandTestController):
    def __init__(self,config,calibration,deployment,link):
        super().__init__(config,calibration,link=link,tracking_error_guard=False,standing_pose_guard=False)
        self.deployment=deployment
        if deployment.get('standpose_file'):
            pose=json.loads(Path(deployment['standpose_file']).read_text())
            if pose['joint_order']!=self.kin.joint_names:raise ValueError('Standpose joint order mismatch')
            if abs(pose['height_m']-deployment['body_height_m'])>1e-9:raise ValueError('Standpose height mismatch')
            q=np.asarray(pose['nominal_joint_position'],dtype=float)
            targets=self.kin.forward(q)
        else:
            targets=np.array([[.145,.13,-deployment['body_height_m']],[.145,-.13,-deployment['body_height_m']],[-.145,.13,-deployment['body_height_m']],[-.145,-.13,-deployment['body_height_m']]])
            q,errors=self.kin.solve(targets,self.kin.stand_q,max_iterations=100)
            if max(errors)>.0005:raise ValueError('Requested standing posture is unreachable')
        self.kin.stand_q=q;self._q=q.copy();self.sequence=0;self.last_command=None;self.history=deque(maxlen=4000)
        self.stand_completed=False;self.walk_table=self.prepare_trajectory(targets)
        self.enable_alignment=None
        self.zero_alignment=None;self._pose_mapping_lock=threading.RLock()

    def move_to_pose(self,goal,seconds,name):
        with self._pose_mapping_lock:
            self._active(self._generation,self._cancel)
            if name=='zero':
                if not np.array_equal(np.asarray(goal),np.zeros(12)):
                    raise ValueError('Zero operation requires the calibrated model-zero target')
                rows=self._feedback(require_enabled=True)
                aligned,events=align_encoder_branches(rows,self.calibration,self.kin.limits)
                encode_model_mit(np.zeros(12),np.zeros(12),self.kp,self.kd,np.zeros(12),aligned,self.config['motor']['mapping_ranges'])
                self.calibration=aligned
                self.zero_alignment=dict(time_monotonic=now(),motors=events,source='Nearest calibrated zero before zero trajectory')
            return super().move_to_pose(goal,seconds,name)

    def _enable(self,token,cancel):
        self._active(token,cancel)
        if self._enabled:return
        if not self.dry_run:
            problems=calibration_problems(self.config,self.calibration,self.kin)
            if problems:raise ValueError('Calibration/configuration invalid: '+'; '.join(problems))
        rows=self._feedback();_,_,count_before=self.link.snapshot()
        # Pre-enable CAN snapshots can describe yesterday's pose. Their position
        # must never become a nonzero-gain holding target after a power cycle.
        self.link.send(encode_mit({r['id']:(r['position'],0.,0.,0.,0.) for r in rows}),cancel)
        self.link.send(GLOBAL_ENABLE,cancel);deadline=now()+1.5
        while True:
            if cancel.wait(self.dt):raise Cancelled('Enable cancelled')
            self._active(token,cancel);rows=self._feedback();_,_,count=self.link.snapshot()
            if count>count_before and all(r['state']==1 for r in rows):break
            if now()>deadline:raise RuntimeError('No updated enabled motor feedback')
        aligned,events=align_encoder_branches(rows,self.calibration,self.kin.limits)
        q=feedback_to_model(rows,aligned)[0]
        # Validate wire encoding before committing the session branch mapping.
        encode_model_mit(q,np.zeros(12),self.kp,self.kd,np.zeros(12),aligned,self.config['motor']['mapping_ranges'])
        self.calibration=aligned
        self.enable_alignment=dict(time_monotonic=now(),motors=events,
                                   source='Post-enable state=1 feedback, before gain ramp')
        self._say('使能后反馈已更新并对齐圈数；从实际当前位置缓升增益',token)
        start=now()
        while True:
            self._active(token,cancel);elapsed=now()-start
            self._send_targets(q,np.zeros(12),cancel,gain=min(elapsed/self.options['gain_ramp_seconds'],1.))
            if elapsed>=self.options['gain_ramp_seconds']:break
            if cancel.wait(self.dt):raise Cancelled('Gain ramp cancelled')
        with self._lock:
            self._active(token,cancel);self._enabled=True;self._mode='hold'
            self._plan=dict(kind='hold',q=q.copy());self._q=q.copy();self._last_tick=now()
        self._say('已使能，保持使能后实测位置',token)

    def status(self):
        result=super().status();result['enable_alignment']=self.enable_alignment
        result['zero_alignment']=self.zero_alignment
        return result

    def prepare_trajectory(self,neutral):
        profile=dict(self.config['sim_candidate']);profile.update(self.options['gaits']['walk'])
        profile.update(settle_seconds=0.,ramp_seconds=self.deployment['walk_ramp_seconds'],mode_profiles={})
        gait=DogGait(profile,neutral,'walk');duration=self.deployment['walk_seconds'];times=np.arange(0,duration+self.dt/2,self.dt)
        if duration<=2*profile['ramp_seconds']:raise ValueError('Walk interval must include ramp up/down and a steady interval')
        positions=[];maximum_error=0.;seed=self.kin.stand_q.copy()
        for t in times:
            feet=gait.targets(float(t));fade=minimum_jerk((duration-t)/profile['ramp_seconds'])[0]
            feet=neutral+(feet-neutral)*fade
            seed,error=self.kin.solve(feet,seed,max_iterations=50)
            maximum_error=max(maximum_error,float(error.max()));positions.append(seed.copy())
        q=np.array(positions);q[0]=q[-1]=self.kin.stand_q
        dq=np.gradient(q,self.dt,axis=0);dq[0]=dq[-1]=0.
        if maximum_error>.001:raise ValueError(f'IK trajectory residual {maximum_error}m')
        for a,b in zip(q,dq):encode_model_mit(a,b,self.kp,self.kd,np.zeros(12),self.calibration,self.config['motor']['mapping_ranges'])
        phases=(times[:,None]*profile['frequency']+np.array(profile['phase_offsets']))%1
        support=phases<profile['duty_factor']
        if support.sum(1).min()<3:raise ValueError('Four-beat walk must plan at least three support feet')
        self.preflight={'imu_used':False,'body_height_target_m':self.deployment['body_height_m'],'walk_seconds':duration,'control_dt':self.dt,
                        'frequency_hz':profile['frequency'],'duty_factor':profile['duty_factor'],'phase_offsets':profile['phase_offsets'],
                        'forward_speed_command_m_s':profile['forward_speed'],'foot_lift_m':profile['step_height'],
                        'max_ik_residual_m':maximum_error,'max_target_joint_speed_rad_s':float(np.abs(dq).max()),
                        'min_planned_support_feet':int(support.sum(1).min()),'kp':self.kp.tolist(),'kd':self.kd.tolist(),
                        'note':'Kinematic trajectory feasibility only; support is planned, physical contact and body height are not measured.'}
        return times,q,dq

    def _feedback(self,require_enabled=False):
        if self.link.error:raise RuntimeError('Feedback reader failed: '+self.link.error)
        rows,stamp,_=self.link.snapshot()
        if rows is None or now()-stamp>self.options['feedback_timeout']:raise RuntimeError('USB feedback missing/stale')
        if len(rows)!=12 or {r['id'] for r in rows}!=set(range(1,13)):raise RuntimeError('Incomplete motor feedback')
        if all(all(r[k]==0 for k in ('state','position','velocity','torque','tmos','tcoil')) for r in rows):
            raise RuntimeError('All motor fields are zero: uninitialized CAN cache, not a measured starting posture')
        for r in rows:
            if not r['valid'] or r['state'] not in (0,1):raise RuntimeError(f'Driver invalid/fault state: ID {r["id"]}, state {r["state"]}')
            if require_enabled and r['state']!=1:raise RuntimeError(f'Motor {r["id"]} no longer enabled')
            if not all(np.isfinite(r[k]) for k in ('position','velocity','torque','tmos','tcoil')):raise RuntimeError('Nonfinite feedback')
        return rows

    def disable(self,reason='Operator disable'):
        self.stand_completed=False
        return super().disable(reason)

    def _send_targets(self,q,dq,cancel,*,gain=1.):
        if cancel.is_set():raise Cancelled('Command cancelled')
        actual,velocity,torque=feedback_to_model(self._feedback(require_enabled=True),self.calibration)
        kp,kd=self.kp*gain,self.kd*gain
        self.link.send(encode_model_mit(q,dq,kp,kd,np.zeros(12),self.calibration,self.config['motor']['mapping_ranges']),cancel)
        self.sequence+=1
        self.last_command={'sequence':self.sequence,'time_monotonic':now(),'q_des':np.asarray(q).tolist(),'dq_des':np.asarray(dq).tolist(),
                           'kp':kp.tolist(),'kd':kd.tolist(),'tau_ff':[0.]*12,'q_actual':actual.tolist(),'dq_actual':velocity.tolist(),
                           'measured_torque_model_nm':torque.tolist(),'tracking_error_rad':(q-actual).tolist()}
        self.history.append(self.last_command)

    def _begin_motion(self,action,token,cancel):
        if action=='trot':raise ValueError('This deployment is four-beat walk')
        if action=='walk':
            self._active(token,cancel)
            if not self._enabled or not self.stand_completed:raise ValueError('Complete stand before walk')
            self._feedback(require_enabled=True);stamp=now()
            with self._lock:self._active(token,cancel);self._plan={'kind':'ik_walk','start':stamp};self._mode='walk';self._last_tick=stamp
            return
        if action=='stand':self.stand_completed=False
        return super()._begin_motion(action,token,cancel)

    def _tick(self,token,cancel):
        with self._pose_mapping_lock:return self._tick_with_mapping(token,cancel)

    def _tick_with_mapping(self,token,cancel):
        with self._lock:plan=self._plan
        if not plan or plan['kind']!='ik_walk':
            result=super()._tick(token,cancel)
            if plan and plan['kind']=='stand' and self._plan and self._plan['kind']=='hold':self.stand_completed=True
            return result
        self._active(token,cancel);stamp=now()
        if stamp-self._last_tick>self.options['max_control_gap']:raise RuntimeError('Control loop stalled')
        t=min(stamp-plan['start'],self.deployment['walk_seconds']);ts,qs,dqs=self.walk_table
        q=np.array([np.interp(t,ts,qs[:,i]) for i in range(12)]);dq=np.array([np.interp(t,ts,dqs[:,i]) for i in range(12)])
        self._send_targets(q,dq,cancel)
        with self._lock:
            self._active(token,cancel);self._q=q.copy();self._last_tick=stamp
            if t>=self.deployment['walk_seconds']:self._plan={'kind':'hold','q':self.kin.stand_q.copy()};self._mode='walk_done'
