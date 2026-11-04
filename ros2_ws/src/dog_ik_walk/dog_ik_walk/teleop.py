"""Continuous velocity walk; only the existing controller worker writes motors."""
import hashlib
import json
import math
from pathlib import Path
import threading
import numpy as np
from dog_clock import now
from dog_runtime import minimum_jerk
from dog_protocol import encode_model_mit
from .imu_controller import ImuWalkController


class VelocityTable:
    def __init__(self, controller):
        c=controller;self.settings=c.deployment['teleop']
        profile=dict(c.config['sim_candidate']);profile.update(c.options['gaits']['walk'])
        self.frequency=profile['frequency'];self.vmax=self.settings['max_forward_m_s'];self.wmax=c.deployment['heading']['trajectory_yaw_rate_rad_s']
        self.count=128;self.period=1/self.frequency
        neutral=c.kin.forward(c.kin.stand_q)
        signature=hashlib.sha256(json.dumps(dict(profile=profile,neutral=neutral.tolist(),count=self.count,
            vmax=self.vmax,wmax=self.wmax,geometry=c.kin.geometry,source=Path(__file__).read_text()),sort_keys=True).encode()).hexdigest()
        cache=Path(self.settings['trajectory_cache'])
        if cache.exists():
            with np.load(cache,allow_pickle=False) as saved:
                if str(saved['signature'])!=signature:raise ValueError('Teleop cache mismatch')
                self.q=saved['q'].copy();self.residual=float(saved['residual'])
            return
        self.q=np.empty((3,3,self.count,12));self.residual=0.
        for vi,v in enumerate((-self.vmax,0.,self.vmax)):
            for wi,w in enumerate((-self.wmax,0.,self.wmax)):
                seed=c.kin.stand_q.copy()
                for k in range(self.count):
                    phase=(k/self.count+np.array(profile['phase_offsets']))%1
                    feet=neutral.copy();duty=profile['duty_factor']
                    for leg,p in enumerate(phase):
                        if p<duty:
                            u=p/duty;s=duty/self.frequency*(.5-u*u*(3-2*u))
                        else:
                            u=(p-duty)/(1-duty);s=duty/self.frequency*(-.5+u*u*(3-2*u))
                            feet[leg,2]+=profile['step_height']*math.sin(math.pi*u)**2
                        a=w*s;rot=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
                        shift=v*s*np.array([np.sinc(a/np.pi),.5*a*np.sinc(a/(2*np.pi))**2])
                        feet[leg,:2]=rot@neutral[leg,:2]+shift
                    seed,error=c.kin.solve(feet,seed,max_iterations=50)
                    self.residual=max(self.residual,float(max(error)));self.q[vi,wi,k]=seed
        if self.residual>.001:raise ValueError('Teleop IK residual exceeds 1 mm')
        cache.parent.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(cache,q=self.q,residual=self.residual,signature=signature)

    def sample(self,phase,v,w,dv,dw):
        x=(phase%1)*self.count;k=int(x);a=x-k
        banks=self.q[:,:,k]*(1-a)+self.q[:,:,(k+1)%self.count]*a
        speeds=(self.q[:,:,(k+1)%self.count]-self.q[:,:,k])*self.count*self.frequency
        vi=1 if v>=0 else 0;wi=1 if w>=0 else 0
        av=float(np.clip(v/self.vmax+(0 if v>=0 else 1),0,1));aw=float(np.clip(w/self.wmax+(0 if w>=0 else 1),0,1))
        def mix(b):return (1-av)*((1-aw)*b[vi,wi]+aw*b[vi,wi+1])+av*((1-aw)*b[vi+1,wi]+aw*b[vi+1,wi+1])
        q=mix(banks);dq=mix(speeds)
        dq+=dv/self.vmax*((1-aw)*(banks[vi+1,wi]-banks[vi,wi])+aw*(banks[vi+1,wi+1]-banks[vi,wi+1]))
        dq+=dw/self.wmax*((1-av)*(banks[vi,wi+1]-banks[vi,wi])+av*(banks[vi+1,wi+1]-banks[vi+1,wi]))
        return q,dq


class TeleopController(ImuWalkController):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.velocity_table=VelocityTable(self);self.teleop_lock=threading.RLock()
        self.drive=None;self.blocked_gesture=None;self.telemetry={'state':'idle'}
        self.v=0.;self.w=0.;self.amplitude=0.;self.ramp=None;self.steering_mode=None
        self.velocity_source='joystick'

    def select_velocity_source(self,source):
        if source not in ('joystick','navigation'):raise ValueError('Unknown velocity source')
        with self.teleop_lock:
            with self._lock:
                if self._mode in ('teleop','teleop_stopping') or (self.drive and self.drive['active']):
                    raise RuntimeError('Stop and wait for standing before changing velocity source')
            self.velocity_source=source
            self.telemetry['velocity_source']=source

    def accept_velocity(self,data,source='joystick'):
        stamp=now();settings=self.deployment['teleop']
        if type(data.get('active')) is not bool:raise ValueError('active must be boolean')
        gesture=data.get('gesture');seq=data.get('seq');issued=data.get('issued_monotonic')
        if not isinstance(gesture,str) or not 1<=len(gesture)<=100 or type(seq) is not int or seq<0:raise ValueError('Invalid command identity')
        if not isinstance(issued,(int,float)) or not math.isfinite(issued) or not 0<=stamp-issued<=settings['command_timeout_s']:raise ValueError('Expired velocity command')
        v=float(data.get('vx',0));w=float(data.get('wz',0))
        if not math.isfinite(v+w) or abs(v)>settings['max_forward_m_s']+1e-9 or abs(w)>self.velocity_table.wmax+1e-9:raise ValueError('Velocity outside configured gait range')
        with self.teleop_lock:
            if source!=getattr(self,'velocity_source','joystick'):raise RuntimeError('Velocity source is not selected: '+source)
            if self.drive and gesture==self.drive['gesture'] and seq<=self.drive['seq']:raise ValueError('Out-of-order velocity command')
            if gesture==self.blocked_gesture and data['active']:raise ValueError('Release joystick and start a new gesture')
            if not data['active']:
                if self.drive and gesture==self.drive['gesture']:self.stop_velocity('released')
                return
            with self._lock:
                if not self._enabled or not self.stand_completed or self._disabling or self._commands:raise RuntimeError('Complete stand before joystick control')
                if self._mode not in ('stand','hold','walk_done','teleop','teleop_stopping'):raise RuntimeError('Another motion is running')
                fresh_plan=not self._plan or self._plan['kind']!='teleop'
            mode='navigation' if source=='navigation' else ('turn' if abs(w)>1e-6 else ('forward' if v>=0 else 'backward'))
            if not self.drive or gesture!=self.drive['gesture'] or mode!=self.steering_mode:
                if source=='joystick':self.heading.activate_command(self.heading.capture_command('joystick_'+mode,stamp))
                self.heading.rate=self.w
                self.steering_mode=mode
            self.drive=dict(gesture=gesture,seq=seq,vx=v,wz=w,received=stamp,active=abs(v)+abs(w)>1e-6)
            if fresh_plan:
                self.v=self.w=self.amplitude=0.;self.ramp=None
                self.heading.rate=0.
                with self._lock:
                    self._active(self._generation,self._cancel)
                    self._plan=dict(kind='teleop',start=stamp);self._mode='teleop';self._last_tick=stamp
            self.telemetry.update(state='accepted',gesture=gesture,seq=seq,requested_vx=v,requested_wz=w,velocity_source=source)

    def stop_velocity(self,reason='released'):
        with self.teleop_lock:
            if self.drive:
                self.blocked_gesture=self.drive['gesture'];self.drive['active']=False
            self.telemetry['state']=reason

    def disable(self,reason='Operator disable'):
        # Super's cancellation latch wins even if a trajectory tick is busy.
        result=super().disable(reason)
        if hasattr(self,'teleop_lock'):self.stop_velocity('disabled')
        return result

    def status(self):
        s=super().status();s['stand_ready']=self.stand_completed
        if hasattr(self,'teleop_lock'):
            with self.teleop_lock:
                s['teleop']=dict(self.telemetry)
                s['teleop']['velocity_source']=getattr(self,'velocity_source','joystick')
        return s

    def _tick(self,token,cancel):
        with self._lock:plan=self._plan
        if not plan or plan['kind']!='teleop':return super()._tick(token,cancel)
        with self.teleop_lock:
            self._active(token,cancel);stamp=now();dt=stamp-self._last_tick
            if dt>self.options['max_control_gap']:raise RuntimeError('Control loop stalled')
            cfg=self.deployment['teleop'];drive=self.drive
            if drive and stamp-drive['received']>cfg['command_timeout_s'] and drive['active']:self.stop_velocity('command_timeout')
            active=bool(drive and drive['active']);goal=1. if active else 0.
            if self.ramp is None or self.ramp['goal']!=goal:self.ramp=dict(start=stamp,q0=self.amplitude,goal=goal)
            blend,slope=minimum_jerk((stamp-self.ramp['start'])/self.deployment['walk_ramp_seconds'])
            amp=self.ramp['q0']+(goal-self.ramp['q0'])*blend
            damp=(goal-self.ramp['q0'])*slope/self.deployment['walk_ramp_seconds'];self.amplitude=amp
            vtarget=drive['vx'] if active else 0.;oldv=self.v
            self.v+=(vtarget-self.v)*dt/(cfg['velocity_filter_seconds']+dt);dv=(self.v-oldv)/dt
            if active and self.steering_mode not in ('turn','navigation'):self.w,dw=self.heading.step(dt,stamp)
            else:
                target=drive['wz'] if active else 0.;oldw=self.w
                self.w+=(target-self.w)*dt/(self.deployment['heading']['command_filter_seconds']+dt);dw=(self.w-oldw)/dt
                self.heading.rate=self.w
            q,dq=self.velocity_table.sample((stamp-plan['start'])*self.velocity_table.frequency,self.v,self.w,dv,dw)
            delta=q-self.kin.stand_q;dq=amp*dq+damp*delta;q=self.kin.stand_q+amp*delta
            self._send_targets(q,dq,cancel)
            self.telemetry.update(active=active,amplitude=amp,vx=self.v,wz=self.w,command_age_s=None if drive is None else stamp-drive['received'])
            self.last_command.update(imu_used=True,teleop=dict(self.telemetry),heading=dict(self.heading.detail))
            with self._lock:
                self._active(token,cancel);self._q=q.copy();self._last_tick=stamp
                self._mode='teleop' if active else 'teleop_stopping'
                if not active and amp==0.:
                    self._plan=dict(kind='hold',q=self.kin.stand_q.copy());self._mode='hold'
                    self.telemetry.update(state='standing',active=False)
