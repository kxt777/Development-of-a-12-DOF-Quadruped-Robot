"""Plot recorded Nav2 software test evidence, not an invented walking result."""
import json
from pathlib import Path
import numpy as np
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Circle

root=Path(__file__).resolve().parents[2]/'dog_deployments/navigation_20261021'
ev=root/'nav_integration_20261021'
rows=[json.loads(line) for line in (ev/'dynamic_obstacle_test/trace.jsonl').read_text().splitlines()]
summary=json.loads((ev/'summary.json').read_text())
route=json.loads((root/'dog_fire_grid_20261021_manual_v1/reference_route.json').read_text())
ref=np.array(route['points_xy_m']);indices=[0,4,8,11,15,18,21,25]
duration=summary['dynamic_elapsed_s']
samples=[r for r in rows if r['kind']=='sample' and 0<r['elapsed']<=duration]
actual=np.array([[r['x'],r['y']] for r in samples]);t=np.array([r['elapsed'] for r in samples])
initial=np.array(next(r['xy'] for r in rows if r['kind']=='path'))
font=FontProperties(fname='C:/Windows/Fonts/msyh.ttc')
plt.rcParams['axes.unicode_minus']=False
fig=plt.figure(figsize=(11,9),layout='constrained');gs=fig.add_gridspec(2,1,height_ratios=[4,1])
ax=fig.add_subplot(gs[0]);im=np.array(Image.open(root/'dog_fire_grid_20261021_manual_v1/map.pgm'))
ax.imshow(im,cmap='gray',vmin=0,vmax=254,extent=[-11.3,-11.3+850*.05,-8,-8+438*.05],origin='upper')
ax.plot(ref[:,0],ref[:,1],color='#007ebd',lw=2,ls='--',label='用户蓝线参考路线')
ax.plot(initial[:,0],initial[:,1],color='#aaa',lw=1.5,label='障碍出现前的 Nav2 规划')
ax.plot(actual[:,0],actual[:,1],color='#913ab3',lw=2.5,label='加入障碍后的软件测试轨迹')
ax.add_patch(Circle((1.25,.38),.20,color='#e94545',alpha=.8,label='第 8 秒出现的模拟障碍'))
ax.scatter(ref[indices,0],ref[indices,1],s=45,color='#007ebd',zorder=6)
for j,i in enumerate(indices):ax.annotate(str(j+1),ref[i],xytext=(5,8),textcoords='offset points',color='#00517c',weight='bold')
ax.scatter(actual[0,0],actual[0,1],marker='>',s=90,color='#00a166',zorder=7)
ax.scatter(actual[-1,0],actual[-1,1],marker='s',s=65,color='#00a166',zorder=7)
perturb=actual[np.argmin(abs(t-20.05))]
ax.annotate('第 20 秒人为加入偏移与偏航',xy=perturb,xytext=(-.05,.73),fontproperties=font,
            fontsize=10,color='#555',arrowprops={'arrowstyle':'->','color':'#777'})
ax.set(xlim=(-1.25,3.25),ylim=(-1.85,1.1),xlabel='X / m',ylabel='Y / m',aspect='equal')
ax.set_title('预设路线 + 实时重规划 + 避障：软件联调通过',fontproperties=font,fontsize=18,pad=12)
ax.legend(prop=font,loc='lower left',fontsize=10,framealpha=.97)
ax.grid(alpha=.12)
speed=fig.add_subplot(gs[1]);speed.plot(t,[r['vx'] for r in samples],label='v (m/s)',color='#007ebd')
speed.plot(t,[r['wz'] for r in samples],label='yaw rate (rad/s)',color='#913ab3')
speed.axvline(8,color='#e94545',ls='--',lw=1);speed.axvline(20,color='#888',ls=':',lw=1)
speed.set(xlabel='Elapsed / s',xlim=(0,duration),ylim=(-.17,.17));speed.legend(ncol=2,loc='lower right');speed.grid(alpha=.2)
fig.suptitle(f'8 个途经点 · Nav2 成功完成 · {duration:.1f} 秒\n隔离的点机器人运动学测试；未驱动真机，未验证整机通过间隙',fontproperties=font,fontsize=11)
fig.savefig(ev/'navigation_validation.png',dpi=150)
plt.close(fig)
