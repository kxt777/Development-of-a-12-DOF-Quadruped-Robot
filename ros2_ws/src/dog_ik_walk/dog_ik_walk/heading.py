"""IMU mounting and continuous yaw correction. No ROS or hardware dependencies."""
import math
import threading
import time
import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def rotation_xyzw(q):
    x,y,z,w=np.asarray(q,dtype=float)
    norm=float(np.linalg.norm([x,y,z,w]))
    if not np.isfinite(norm) or abs(norm-1)>0.1:raise ValueError('Invalid IMU quaternion')
    x,y,z,w=np.array([x,y,z,w])/norm
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def mounted_sample(q_sensor, gyro_sensor, accel_sensor):
    mount=np.diag([1.,-1.,-1.])
    r=rotation_xyzw(q_sensor)@mount.T
    gyro=mount@np.asarray(gyro_sensor); accel=mount@np.asarray(accel_sensor)
    if not np.isfinite(np.r_[gyro,accel]).all():raise ValueError('Nonfinite IMU vector')
    x,y,z=gyro; cross=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
    dr=r@cross
    denominator=r[0,0]**2+r[1,0]**2
    if denominator<1e-6:raise ValueError('Heading undefined near vertical body orientation')
    yaw=math.atan2(r[1,0],r[0,0])
    yaw_rate=(r[0,0]*dr[1,0]-r[1,0]*dr[0,0])/denominator
    # Hamilton q_WS * inverse(q_BS), q_BS=(x=1,y=0,z=0,w=0).
    sx,sy,sz,sw=np.asarray(q_sensor)/np.linalg.norm(q_sensor)
    body_q=np.array([-sw,-sz,sy,sx])
    return dict(yaw=yaw,yaw_rate=float(yaw_rate),quaternion_xyzw=body_q.tolist(),
                gyro_body=gyro.tolist(),accel_body=accel.tolist())


class HeadingHold:
    def __init__(self, settings):
        values=[settings[k] for k in ('kp','kd','command_filter_seconds','trajectory_yaw_rate_rad_s','max_imu_age_s')]
        if not np.isfinite(values).all() or min(values)<0 or min(values[2:])<=0:
            raise ValueError('Heading gains must be finite nonnegative, time constants/envelope positive')
        self.settings=settings; self.lock=threading.RLock(); self.latest=None
        self.reference=None; self.rate=0.; self.detail={}; self.command=None; self.command_sequence=0
        self.gap_count=0; self.in_gap=False
        self.filtered_yaw_rate=None
        self.response=settings.get('straight_response')
        if self.response is not None:
            keys=('full_correction_error_deg','braking_error_deg','yaw_rate_filter_seconds','command_filter_seconds')
            v=[self.response[k] for k in keys]
            if not np.isfinite(v).all() or min(v)<=0 or v[1]>v[0]:
                raise ValueError('Invalid straight heading response settings')

    def receive(self,q,gyro,accel,stamp=None):
        sample=mounted_sample(q,gyro,accel)
        sample['receipt_monotonic']=time.monotonic() if stamp is None else stamp
        with self.lock:self.latest=sample
        return sample

    def sample(self,stamp=None):
        stamp=time.monotonic() if stamp is None else stamp
        with self.lock: sample=None if self.latest is None else dict(self.latest)
        if sample is None or not 0<=stamp-sample['receipt_monotonic']<=self.settings['max_imu_age_s']:
            raise RuntimeError('No fresh IMU orientation/gyro for heading correction')
        return sample

    def capture_command(self,source,stamp=None):
        """Snapshot the latest fresh mounted heading at external command receipt."""
        stamp=time.monotonic() if stamp is None else stamp
        sample=self.sample(stamp)
        return dict(source=source,received_monotonic=stamp,
                    imu_receipt_monotonic=sample['receipt_monotonic'],target_yaw_rad=sample['yaw'])

    def activate_command(self,command):
        with self.lock:
            self.command_sequence+=1
            self.command=dict(command,sequence=self.command_sequence)
            self.reference=command['target_yaw_rad']; self.rate=0.; self.detail={}
            self.filtered_yaw_rate=None

    def reset(self,stamp=None):
        # Retained for offline callers; real commands explicitly carry their source.
        self.activate_command(self.capture_command('explicit_reset',stamp))

    def require_command(self):
        with self.lock:
            if self.command is None:raise RuntimeError('No heading captured at command receipt')

    def step(self,dt,stamp=None):
        with self.lock:return self._step_locked(dt,stamp)

    def _step_locked(self,dt,stamp=None):
        if self.reference is None:raise RuntimeError('No heading captured at command receipt')
        stamp=time.monotonic() if stamp is None else stamp
        if self.command.get('source') in ('demo_right','demo_left','walk_right','walk_left'):
            # Explicit finite steering test: measure IMU yaw without steering
            # back toward the initial straight-line heading.
            try:sample=self.sample(stamp)
            except RuntimeError:sample=None
            direction='right' if self.command['source'].endswith('_right') else 'left'
            target=(-1. if direction=='right' else 1.)*self.settings['trajectory_yaw_rate_rad_s']
            old=self.rate
            self.rate+=(target-self.rate)*dt/(self.settings['command_filter_seconds']+dt)
            self.detail=dict(control_mode='fixed_'+direction+'_turn',heading_reference_rad=self.reference,
                heading_actual_rad=None if sample is None else sample['yaw'],
                heading_error_rad=None,
                yaw_rate_measured_rad_s=None if sample is None else sample['yaw_rate'],
                yaw_rate_requested_rad_s=target,yaw_rate_command_rad_s=self.rate,
                imu_receipt_monotonic=None if self.latest is None else self.latest['receipt_monotonic'],
                imu_age_s=None if self.latest is None else stamp-self.latest['receipt_monotonic'],
                imu_available=sample is not None,heading_command=dict(self.command))
            return self.rate,(self.rate-old)/dt
        try:sample=self.sample(stamp)
        except RuntimeError:
            # Missing measurements do not abort the finite walk. Do not keep
            # steering on stale gyro/yaw: smoothly fade the correction to zero.
            if not self.in_gap:self.gap_count+=1
            self.in_gap=True
            self.filtered_yaw_rate=None
            old=self.rate
            self.rate+=(0.-self.rate)*dt/(self.settings['command_filter_seconds']+dt)
            self.detail=dict(heading_reference_rad=self.reference,heading_actual_rad=None,
                heading_error_rad=None,yaw_rate_measured_rad_s=None,yaw_rate_requested_rad_s=0.,
                yaw_rate_command_rad_s=self.rate,imu_receipt_monotonic=None if self.latest is None else self.latest['receipt_monotonic'],
                imu_age_s=None if self.latest is None else stamp-self.latest['receipt_monotonic'],
                imu_available=False,gap_count=self.gap_count,heading_command=dict(self.command))
            return self.rate,(self.rate-old)/dt
        self.in_gap=False
        error=wrap(self.reference-sample['yaw'])
        requested=self.settings['kp']*error-self.settings['kd']*sample['yaw_rate']
        envelope=self.settings['trajectory_yaw_rate_rad_s']
        response_detail={}
        filter_seconds=self.settings['command_filter_seconds']
        if self.response is not None:
            cfg=self.response
            if self.filtered_yaw_rate is None:self.filtered_yaw_rate=sample['yaw_rate']
            else:self.filtered_yaw_rate+=(sample['yaw_rate']-self.filtered_yaw_rate)*dt/(cfg['yaw_rate_filter_seconds']+dt)
            # 3--5 degrees is an output-strength scale, never a trigger deadband.
            proportional=envelope*float(np.clip(error/math.radians(cfg['full_correction_error_deg']),-1.,1.))
            # Brake near the target. Outside that region, gait-rate oscillations
            # must not cancel an already-needed corrective turn.
            braking_weight=max(0.,1.-abs(error)/math.radians(cfg['braking_error_deg']))
            damping=braking_weight*float(np.clip(self.settings['kd']*self.filtered_yaw_rate,-envelope,envelope))
            requested=proportional-damping
            filter_seconds=cfg['command_filter_seconds']
            response_detail=dict(control_mode='responsive_heading_v1',
                yaw_rate_filtered_rad_s=self.filtered_yaw_rate,
                proportional_yaw_rate_rad_s=proportional,damping_yaw_rate_rad_s=damping,
                braking_weight=braking_weight,command_filter_seconds=filter_seconds)
        target=float(np.clip(requested,-envelope,envelope))
        old=self.rate
        self.rate+=(target-self.rate)*dt/(filter_seconds+dt)
        self.detail=dict(heading_reference_rad=self.reference,heading_actual_rad=sample['yaw'],
                         heading_error_rad=error,yaw_rate_measured_rad_s=sample['yaw_rate'],
                         yaw_rate_requested_rad_s=requested,yaw_rate_command_rad_s=self.rate,
                         imu_receipt_monotonic=sample['receipt_monotonic'],
                         imu_age_s=stamp-sample['receipt_monotonic'],imu_available=True,gap_count=self.gap_count,
                         heading_command=dict(self.command))
        self.detail.update(response_detail)
        return self.rate,(self.rate-old)/dt

    def status(self):
        with self.lock:
            try:sample=self.sample(); fresh=True
            except RuntimeError:sample=None;fresh=False
            return dict(fresh=fresh,reference_rad=self.reference,sample=sample,correction=dict(self.detail),
                        command=None if self.command is None else dict(self.command))
