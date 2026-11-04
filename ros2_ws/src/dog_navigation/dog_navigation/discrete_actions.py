"""Six translating joystick positions, held between slow navigation decisions.

No ROS dependencies: the policy can be replayed against physical-run logs.
Gait generation remains in the existing joystick executor.
"""
import math

NAMES = ('forward', 'forward_left', 'turn_left', 'backward_left',
         'backward', 'backward_right', 'turn_right', 'forward_right')


def joystick_action(index, vmax=.08, wmax=.15):
    # Same unit circle and 8% per-axis dead zone as dog_ik_walk/web/app.js.
    def dead(value):
        return 0. if abs(value) < .08 else math.copysign((abs(value)-.08)/.92, value)
    angle = index * math.pi / 4
    return NAMES[index % 8], dead(math.cos(angle))*vmax, dead(math.sin(angle))*wmax


def quantize(v, w, vmax=.08, wmax=.15):
    if not all(math.isfinite(x) for x in (v, w, vmax, wmax)) or min(vmax,wmax)<=0:
        raise ValueError('Invalid navigation velocity')
    if abs(v)+abs(w)<1e-9:
        return 'stop', 0., 0.
    # Preserve a slow approach's forward/backward intent: sector selection from
    # relative magnitudes would turn (small v, large w) into an endless spin.
    # Reuse the joystick's per-axis dead zone; select signs, never a new angle.
    sv=0. if abs(v/vmax)<.08 else math.copysign(1.,v)
    sw=0. if abs(w/wmax)<.08 else math.copysign(1.,w)
    if sv==0 and sw==0:return 'stop',0.,0.
    # Pure rotation is no longer an executable action. A lateral request uses
    # the existing forward diagonal, never an in-place turn.
    if sv==0:sv=1.
    angle = math.atan2(sw, sv)
    return joystick_action(int(math.floor(angle/(math.pi/4)+.5)) % 8, vmax, wmax)


def path_command(points, base, lookahead=.5, vmax=.08, wmax=.15, approach_sign=None):
    """Select motion from a planned path in the SAME frame as base.

    Use the existing 0.5 m lookahead and joystick's 8% direction deadzone.
    A point behind selects reverse; steering reverses with travel direction.
    No odometry angular-velocity term or high-rate Twist sign is used.
    """
    if not points:raise ValueError('Empty path')
    x,y,yaw=base
    closest=min(range(len(points)),key=lambda i:(points[i][0]-x)**2+(points[i][1]-y)**2)
    carrot=points[closest];remaining=lookahead
    for a,b in zip(points[closest:],points[closest+1:]):
        length=math.hypot(b[0]-a[0],b[1]-a[1])
        if length>=remaining and length>0:
            carrot=(a[0]+(b[0]-a[0])*remaining/length,a[1]+(b[1]-a[1])*remaining/length)
            break
        remaining-=length;carrot=b
    dx,dy=carrot[0]-x,carrot[1]-y
    bx=math.cos(yaw)*dx+math.sin(yaw)*dy
    by=-math.sin(yaw)*dx+math.cos(yaw)*dy
    distance=math.hypot(bx,by)
    if distance<1e-9:return 0.,0.
    travel=1. if bx>=0 else -1.
    # Keep the chosen forward/reverse heading convention for this waypoint.
    # Translation may reverse near a lateral target without undoing yaw progress
    # on every four-second decision (a translating multi-point turn).
    steering=travel if approach_sign is None else approach_sign
    turn=0. if abs(by)/distance<.08 else math.copysign(1.,steering*by)
    return travel*vmax,turn*wmax


class HeldAction:
    def __init__(self, seconds=4., vmax=.08, wmax=.15):
        if seconds<=0:raise ValueError('Positive action duration required')
        self.seconds=seconds;self.vmax=vmax;self.wmax=wmax
        self.action=('stop',0.,0.);self.next_decision=0.;self.revision=0

    def _set(self, action):
        if action!=self.action:self.action=action;self.revision+=1
        return self.action

    def stop(self, now, reset=False):
        if reset:self.next_decision=now
        return self._set(('stop',0.,0.))

    def step(self, now, v, w, interruption=None):
        # An early stop never allows rapid left/right reactivation. The original
        # decision deadline remains in force until it is safe to choose again.
        if interruption or abs(v)+abs(w)<1e-9:
            return self.stop(now)
        if now>=self.next_decision:
            self.next_decision=now+self.seconds
            return self._set(quantize(v,w,self.vmax,self.wmax))
        return self.action


def swept_clear(grid, width, height, resolution, origin, base, v, w,
                duration, footprint=()):
    """Check the HELD action against Nav2's local occupancy costmap.

    99/100 are inscribed/lethal costmap cells. Unknown or out-of-map is not
    certified clear. Costmap origin and base use the same coordinate frame.
    An empty footprint preserves the existing explicit point-robot assumption.
    """
    ox,oy,oa=origin;x,y,yaw=base
    radius=max((math.hypot(*p) for p in footprint),default=0.)
    distance=(abs(v)+abs(w)*radius)*duration
    steps=max(1,math.ceil(distance/(resolution*.5)))
    # Fill the footprint, not just its vertices, so interior occupied cells count.
    points=[(0.,0.)]
    if footprint:
        import numpy as np
        for px in np.arange(min(p[0] for p in footprint),max(p[0] for p in footprint)+resolution*.5,resolution*.5):
            for py in np.arange(min(p[1] for p in footprint),max(p[1] for p in footprint)+resolution*.5,resolution*.5):
                inside=False
                for a,b in zip(footprint,footprint[1:]+footprint[:1]):
                    if (a[1]>py)!=(b[1]>py) and px<(b[0]-a[0])*(py-a[1])/(b[1]-a[1])+a[0]:inside=not inside
                if inside:points.append((px,py))
        points.extend(footprint)
    for i in range(steps+1):
        t=duration*i/steps;theta=yaw+w*t
        if abs(w)<1e-9:bx=x+v*t*math.cos(yaw);by=y+v*t*math.sin(yaw)
        else:bx=x+v/w*(math.sin(theta)-math.sin(yaw));by=y-v/w*(math.cos(theta)-math.cos(yaw))
        for px,py in points:
            dx=bx+math.cos(theta)*px-math.sin(theta)*py-ox
            dy=by+math.sin(theta)*px+math.cos(theta)*py-oy
            gx=math.floor((math.cos(oa)*dx+math.sin(oa)*dy)/resolution)
            gy=math.floor((-math.sin(oa)*dx+math.cos(oa)*dy)/resolution)
            if not (0<=gx<width and 0<=gy<height):return False
            value=grid[gy*width+gx]
            if value<0 or value>=99:return False
    return True
