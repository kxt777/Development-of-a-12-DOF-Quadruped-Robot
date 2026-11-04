"""Read-only audit of the saved physical navigation trace."""
import json,collections
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

p=Path(r'D:\Project\dog\dog_deployments\navigation_20261021\dog_live_v4_20261021_045921')
rows=[json.loads(l) for l in (p/'mission_trace.jsonl').open()]
c=[r for r in rows if r['kind']=='controller'];b=[r for r in rows if r['kind']=='bridge'];n=[r for r in rows if r['kind']=='command']
start=rows[0]['unix']
def category(v,w):
    if abs(v)<.001:return 'left_turn' if w>.001 else 'right_turn' if w<-.001 else 'zero'
    return ('forward' if v>0 else 'backward')+('_left' if w>.001 else '_right' if w<-.001 else '_straight')
def durations(data,get):
    out=collections.defaultdict(float)
    for a,z in zip(data,data[1:]):out[category(*get(a))]+=z['unix']-a['unix']
    return dict(out)
def flips(data,get):
    values=[get(r) for r in data if abs(get(r))>.02]
    return sum(a*b<0 for a,b in zip(values,values[1:]))
result=dict(source=str(p/'mission_trace.jsonl'),audit_scope='Saved 2026-10-21 physical trial; current NUC SSH unavailable. No actuator commands sent.',
    classification_thresholds=dict(vx_m_s=.001,wz_rad_s=.001,sign_flip_wz_rad_s=.02),
    nav_durations_s=durations(n,lambda r:(r['vx'],r['wz'])),
    bridge_durations_s=durations(b,lambda r:(r['data']['preview_vx'],r['data']['preview_wz'])),
    gait_filtered_input_durations_s=durations(c,lambda r:(r['data']['teleop']['vx'],r['data']['teleop']['wz'])),
    nav_turn_sign_changes=flips(n,lambda r:r['wz']),
    bridge_turn_sign_changes=flips(b,lambda r:r['data']['preview_wz']),
    gait_input_turn_sign_changes_at_status_sample_rate=flips(c,lambda r:r['data']['teleop']['wz']),
    bridge_scaled_messages=sum(r['data']['gait_scale']>1.001 for r in b),bridge_messages=len(b),
    usb_write_counter_increase=c[-1]['data']['usb_commands_written']-c[0]['data']['usb_commands_written'],
    preflight_report=c[0]['data']['trajectory_preflight'],
    limitations=['teleop.vx/wz are filtered gait inputs, not measured body velocities.',
        'Controller status sampled approximately every 0.5s; gait sign count is not full control-rate count.',
        'Motor feedback is cached CAN data without per-motor freshness; full sent joint targets are not in this copied trace.',
        'The cause of upstream yaw-command oscillation is not established by this audit.'])
(p/'gait_execution_audit.json').write_text(json.dumps(result,indent=2))
plt.rcParams['font.family']=FontProperties(fname='C:/Windows/Fonts/msyh.ttc').get_name()
plt.rcParams['axes.unicode_minus']=False
fig,axes=plt.subplots(3,1,figsize=(12,9))
tn=np.array([r['unix']-start for r in n]);tb=np.array([r['unix']-start for r in b]);tc=np.array([r['unix']-start for r in c])
axes[0].plot(tn,[r['vx'] for r in n],color='#228be6',label='Nav2 前进指令')
axes[0].plot(tc,[r['data']['teleop']['vx'] for r in c],color='#e67700',label='步态执行器滤波后输入')
axes[0].set(ylabel='前进速度输入 / m/s',title='实机日志：导航给了什么，步态执行器采用了什么')
axes[1].plot(tb,[r['data']['preview_wz'] for r in b],color='#228be6',lw=.7,label='桥接后的转向指令')
axes[1].plot(tc,[r['data']['teleop']['wz'] for r in c],color='#e67700',lw=1.7,label='步态执行器滤波后输入')
axes[1].set(ylabel='转向速度输入 / rad/s',title='全程转向指令频繁变号')
axes[2].step(tb,[r['data']['preview_wz'] for r in b],where='post',color='#228be6',label='桥接后的转向指令')
axes[2].plot(tc,[r['data']['teleop']['wz'] for r in c],'o-',color='#e67700',ms=3,label='执行器状态（约 2 Hz）')
axes[2].set(xlim=(0,10),xlabel='运动开始后 / s',ylabel='转向速度输入 / rad/s',title='前 10 秒放大：左／右指令交替，滤波后幅度变小')
for ax in axes:ax.grid(alpha=.2);ax.legend(loc='upper right',fontsize=9)
fig.text(.5,.015,'图中均为指令或内部步态输入，不是实测机身速度。没有把状态采样点当作完整电机指令记录。',ha='center',fontsize=10)
fig.tight_layout(rect=[0,.045,1,1]);fig.savefig(p/'gait_execution_audit.png',dpi=150)
print(json.dumps({k:v for k,v in result.items() if k!='preflight_report'},ensure_ascii=False))
