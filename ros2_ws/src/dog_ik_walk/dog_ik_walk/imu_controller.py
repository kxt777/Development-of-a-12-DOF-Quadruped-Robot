"""Last real walk's joint trajectory plus IMU-driven planar turning offsets."""
import hashlib
import json
from pathlib import Path
import numpy as np
from dog_clock import now
from dog_runtime import minimum_jerk
from dog_protocol import encode_model_mit
from dog_gait import smooth
from .controller import IkWalkController
from .heading import HeadingHold


def foot_targets(neutral,profile,t,duration,yaw_rate):
    """Same four-beat timing; relative SE(2) stance stroke v + omega x r."""
    frequency=profile['frequency']; duty=profile['duty_factor']
    ramp=smooth(t/profile['ramp_seconds'])*minimum_jerk((duration-t)/profile['ramp_seconds'])[0]
    phases=(t*frequency+np.asarray(profile['phase_offsets']))%1
    feet=neutral.copy()
    for leg,p in enumerate(phases):
        if p<duty:s=duty/frequency*(.5-smooth(p/duty))
        else:
            u=(p-duty)/(1-duty); s=duty/frequency*(-.5+smooth(u))
            feet[leg,2]+=profile['step_height']*ramp*np.sin(np.pi*u)**2
        a=yaw_rate*s; cs,sn=np.cos(a),np.sin(a)
        rot=np.array([[cs,-sn],[sn,cs]])
        shift=profile['forward_speed']*s*np.array([np.sinc(a/np.pi),.5*a*np.sinc(a/(2*np.pi))**2])
        feet[leg,:2]+=ramp*(rot@neutral[leg,:2]+shift-neutral[leg,:2])
    return feet


class ImuWalkController(IkWalkController):
    def __init__(self,config,calibration,deployment,link):
        super().__init__(config,calibration,deployment,link)
        self.heading=HeadingHold(deployment['heading'])

    def prepare_trajectory(self,neutral):
        profile=dict(self.config['sim_candidate']);profile.update(self.options['gaits']['walk'])
        profile.update(ramp_seconds=self.deployment['walk_ramp_seconds'])
        duration=self.deployment['walk_seconds']; envelope=self.deployment['heading']['trajectory_yaw_rate_rad_s']
        if duration<=2*profile['ramp_seconds'] or envelope<=0:raise ValueError('Invalid walk duration or yaw envelope')
        identity=dict(profile=profile,neutral=neutral.tolist(),stand=self.kin.stand_q.tolist(),
                      dt=self.dt,duration=duration,yaw_envelope=envelope,heading=self.deployment['heading'],geometry=self.kin.geometry,
                      mapping=self.config['motor']['mapping_ranges'],kp=self.kp.tolist(),kd=self.kd.tolist(),
                      source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        signature=hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()
        cache=Path(self.deployment['trajectory_cache']) if self.deployment.get('trajectory_cache') else None
        if cache and cache.exists():
            with np.load(cache,allow_pickle=False) as saved:
                if str(saved['signature'])!=signature:raise ValueError('Trajectory cache/config mismatch; rebuild offline')
                self.banks=saved['q'].copy();self.bank_velocities=saved['dq'].copy();times=saved['t'].copy()
                self.preflight=json.loads(str(saved['preflight']))
            self.bank_times=times
            return times,self.banks[1],self.bank_velocities[1]
        times=np.arange(0,duration+self.dt/2,self.dt);banks=[];residual=0.
        for rate in (-envelope,0.,envelope):
            qs=[];seed=self.kin.stand_q.copy()
            for t in times:
                feet=foot_targets(neutral,profile,float(t),duration,rate)
                seed,error=self.kin.solve(feet,seed,max_iterations=50)
                residual=max(residual,float(error.max()));qs.append(seed.copy())
            q=np.asarray(qs);q[0]=q[-1]=self.kin.stand_q;banks.append(q)
        self.banks=np.asarray(banks);self.bank_velocities=np.gradient(self.banks,self.dt,axis=1)
        self.bank_velocities[:,[0,-1],:]=0.;self.bank_times=times
        if residual>.001:raise ValueError(f'Turn trajectory IK residual {residual} m')
        for bank,velocities in zip(self.banks,self.bank_velocities):
            for q,dq in zip(bank,velocities):
                encode_model_mit(q,dq,self.kp,self.kd,np.zeros(12),self.calibration,self.config['motor']['mapping_ranges'])
        phase=(times[:,None]*profile['frequency']+profile['phase_offsets'])%1
        support=int((phase<profile['duty_factor']).sum(1).min())
        if support<3:raise ValueError('Keep the last real four-beat walk support schedule')
        slope=np.max(abs(np.diff(self.banks,axis=0)/envelope))
        heading_settings=self.deployment['heading']
        filter_seconds=min(heading_settings['command_filter_seconds'],
            heading_settings.get('straight_response',{}).get('command_filter_seconds',heading_settings['command_filter_seconds']))
        max_slew=2*envelope/(filter_seconds+self.dt)
        time_speed=max(float(abs(self.bank_velocities).max()),float(abs(np.diff(self.banks,axis=1)/self.dt).max()))
        self.preflight=dict(imu_used=True,body_height_target_m=self.deployment['body_height_m'],
            frequency_hz=profile['frequency'],duty_factor=profile['duty_factor'],phase_offsets=profile['phase_offsets'],
            forward_speed_command_m_s=profile['forward_speed'],foot_lift_m=profile['step_height'],
            min_planned_support_feet=support,walk_seconds=duration,control_dt=self.dt,
            max_ik_residual_m=residual,max_target_joint_speed_rad_s=time_speed,
            conservative_speed_with_correction_change_rad_s=float(time_speed+slope*max_slew),
            yaw_rate_bank_rad_s=[-envelope,0.,envelope],kp=self.kp.tolist(),kd=self.kd.tolist(),signature=signature,
            note='IK/encoding feasibility, not measured hardware or free-base stability. Heading gains and yaw envelope are controller tuning, not manufacturer ratings.')
        if cache:
            cache.parent.mkdir(parents=True,exist_ok=True)
            np.savez_compressed(cache,t=times,q=self.banks,dq=self.bank_velocities,signature=signature,
                                preflight=json.dumps(self.preflight))
        return times,self.banks[1],self.bank_velocities[1]

    def sample_targets(self,t,rate,derivative):
        envelope=self.deployment['heading']['trajectory_yaw_rate_rad_s']
        k=min(max(int(t/self.dt),0),len(self.bank_times)-2)
        alpha=np.clip((t-self.bank_times[k])/self.dt,0,1)
        banks=self.banks[:,k]*(1-alpha)+self.banks[:,k+1]*alpha
        velocities=self.bank_velocities[:,k]*(1-alpha)+self.bank_velocities[:,k+1]*alpha
        lo,hi=(1,2) if rate>=0 else (0,1)
        mix=rate/envelope if rate>=0 else 1+rate/envelope
        q=(1-mix)*banks[lo]+mix*banks[hi]
        dq=(1-mix)*velocities[lo]+mix*velocities[hi]+(banks[hi]-banks[lo])/envelope*derivative
        return q,dq

    def request(self,action,*,preserve_heading=False,command_received=None):
        if action!='walk':return super().request(action)
        # Commit only accepted commands; the worker cannot consume the heading
        # before the queued command and its receipt-time snapshot are committed.
        with self.heading.lock:
            if preserve_heading:
                self.heading.require_command()
                return super().request(action)
            command=self.heading.capture_command('walk',command_received)
            result=super().request(action)
            self.heading.activate_command(command)
            return result

    def _begin_motion(self,action,token,cancel):
        if action=='walk':self.heading.require_command()
        return super()._begin_motion(action,token,cancel)

    def _tick(self,token,cancel):
        with self._lock:plan=self._plan
        if not plan or plan['kind']!='ik_walk':return super()._tick(token,cancel)
        self._active(token,cancel);stamp=now();elapsed=stamp-self._last_tick
        if elapsed>self.options['max_control_gap']:raise RuntimeError('Control loop stalled')
        t=min(stamp-plan['start'],self.deployment['walk_seconds'])
        rate,derivative=self.heading.step(max(elapsed,1e-6))
        q,dq=self.sample_targets(t,rate,derivative)
        self._send_targets(q,dq,cancel)
        self.last_command.update(imu_used=True,heading=dict(self.heading.detail))
        with self._lock:
            self._active(token,cancel);self._q=q.copy();self._last_tick=stamp
            if t>=self.deployment['walk_seconds']:
                self._plan={'kind':'hold','q':self.kin.stand_q.copy()};self._mode='walk_done'
