"""Plot a completed real run, using the actual map TF recorded during navigation."""
import argparse,json,math
from pathlib import Path
import numpy as np
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('mapdir');a=p.parse_args()
run=Path(a.run);maps=Path(a.mapdir)
result=json.loads((run/'route_result.json').read_text())
rows=[json.loads(l) for l in (run/'mission_trace.jsonl').open()]
poses=[r for r in rows if r['kind']=='measured_pose']
xy=np.array([[r['x'],r['y']] for r in poses])
route=json.loads((maps/'reference_route.json').read_text());ref=np.array(route['points_xy_m'])
anchors=ref[[0,4,8,11,15,18,21,25]]
visited=[];index=0
for r,point in zip(poses,xy):
    if index<len(anchors) and np.linalg.norm(point-anchors[index])<=.35:
        visited.append(dict(target=index+1,elapsed=r['unix']-result['started_unix'],xy=point.tolist()));index+=1
minimum=np.min(np.linalg.norm(xy[:,None,:]-anchors[None,:,:],axis=2),axis=0)
segments=ref[1:]-ref[:-1];v=xy[:,None,:]-ref[:-1]
t=np.clip(np.sum(v*segments,axis=2)/np.sum(segments*segments,axis=1),0,1)
dist=np.min(np.linalg.norm(v-t[:,:,None]*segments,axis=2),axis=1)
summary=dict(version='dog_nav_v4_20261021',localization='fixed initial map alignment + FAST-LIO',
    result=result,elapsed_s=result['finished_unix']-result['started_unix'],
    trajectory_start=xy[0].tolist(),trajectory_end=xy[-1].tolist(),ordered_targets_observed=visited,
    target_min_distances_m=minimum.tolist(),reference_distance_mean_m=float(dist.mean()),
    reference_distance_max_m=float(dist.max()),
    note='Sensor-estimated trajectory, not external ground truth. Distance includes entry and turning segments; not necessarily a completed route.')
(run/'summary.json').write_text(json.dumps(summary,indent=2))
(run/'trajectory.json').write_text(json.dumps(poses))
font=Path('C:/Windows/Fonts/msyh.ttc')
if font.exists():plt.rcParams['font.family']=FontProperties(fname=str(font)).get_name()
plt.rcParams['axes.unicode_minus']=False
im=np.array(Image.open(maps/'map.pgm'))
fig,ax=plt.subplots(figsize=(12,8))
ax.imshow(im,cmap='gray',vmin=0,vmax=255,extent=[-11.3,-11.3+im.shape[1]*.05,-8,-8+im.shape[0]*.05])
ax.plot(ref[:,0],ref[:,1],'--',color='#1488ed',lw=2,label='期望路线')
ax.plot(xy[:,0],xy[:,1],color='#159453',lw=2,label='本次雷达＋IMU 轨迹（控制实际使用）')
for i,(x,y) in enumerate(anchors,1):
    ax.add_patch(plt.Circle((x,y),.35,color='#1488ed',alpha=.08));ax.scatter(x,y,c='#1488ed',s=25)
    ax.annotate(str(i),(x,y),xytext=(5,7),textcoords='offset points',color='#12559e')
ax.scatter(*xy[0],marker='o',s=100,facecolors='white',edgecolors='#159453',zorder=5,label='实测起点')
ax.scatter(*xy[-1],marker='s',s=65,c='#159453',zorder=5,label='实测终点')
for r in poses[::max(1,len(poses)//12)]:
    ax.arrow(r['x'],r['y'],.12*math.cos(r['yaw']),.12*math.sin(r['yaw']),head_width=.045,color='#0f7040',length_includes_head=True)
ax.set(xlim=(min(-1.7,float(xy[:,0].min())-.4),max(3.3,float(xy[:,0].max())+.4)),
    ylim=(min(-2.,float(xy[:,1].min())-.4),max(1.2,float(xy[:,1].max())+.4)),
    xlabel='地图 X / m',ylabel='地图 Y / m',title='真机导航 v4：固定起点对齐，FAST-LIO 更新位置')
ax.set_aspect('equal');ax.legend(loc='best')
status='完成' if result.get('success') else '未完成'
fig.text(.5,.065,f"运行 {summary['elapsed_s']:.1f} 秒；任务{status}；记录中按顺序进入 {len(visited)}/8 个目标范围。",ha='center')
fig.text(.5,.025,'绿线为实机传感器估计，非外部测量真值；包含起点接入和转向段。',ha='center',color='#555555')
fig.tight_layout(rect=[0,.10,1,.98]);fig.savefig(run/'path_comparison.png',dpi=160)
print(json.dumps(summary,ensure_ascii=False))
