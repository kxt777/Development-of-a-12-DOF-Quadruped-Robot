"""Perspective view of measured cloud and actual Nav2 path, with no synthetic scan."""
import json
import sys
from pathlib import Path
import numpy as np
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root / 'dog_navigation/dog_navigation'))
from map_preview import read_pcd

out = root / 'dog_deployments/navigation_20261021'
cloud = read_pcd(out / 'floor_capture_map.pcd')
cloud = cloud[np.isfinite(cloud).all(axis=1)][::3]
path = json.loads((out / 'dog_plan_20261021_01/path.json').read_text())
grid = np.flipud(np.asarray(Image.open(out / 'dog_grid_20261021_01/map.pgm')))
font = FontProperties(fname='C:/Windows/Fonts/msyh.ttc')
fig = plt.figure(figsize=(16,8), facecolor='#10151d')
ax = fig.add_subplot(121, projection='3d', facecolor='#10151d')
ax.scatter(*cloud.T, c=cloud[:,2], cmap='turbo', s=.65, linewidths=0, depthshade=False)
ax.set_box_aspect(np.maximum(np.ptp(cloud,axis=0), .1))
ax.view_init(35,45)
ax.set_title('实测三维点云 · 按高度着色',fontproperties=font,color='white',pad=20,fontsize=17)
ax2 = fig.add_subplot(122,projection='3d',facecolor='#10151d',computed_zorder=False)
h,w = grid.shape
x,y = np.meshgrid(-1.9+np.arange(w+1)*.05,-5.45+np.arange(h+1)*.05)
colors=np.empty((h,w,4)); colors[:]=[.16,.19,.23,1]
colors[grid==254]=[.76,.78,.8,1]
colors[grid==0]=[.02,.025,.03,1]
ax2.plot_surface(x,y,np.zeros_like(x),facecolors=colors, rstride=1,cstride=1,shade=False,antialiased=False,zorder=1)
points=np.array(path['points'])
ax2.plot(points[:,0],points[:,1],np.full(len(points),.07),color='#a1f94b',lw=3,zorder=10)
sx,sy=path['start']; gx,gy=path['goal']
ax2.scatter([sx],[sy],[.08],color='#39d9ef',s=60,depthshade=False,zorder=11)
ax2.quiver(gx,gy,.08,.55,0,0,color='#ff4242',linewidth=2,arrow_length_ratio=.3,zorder=11)
ax2.text(sx-.5,sy-.6,.1,'起点',fontproperties=font,color='#007584',zorder=12)
ax2.text(gx+.65,gy,.1,'目标',fontproperties=font,color='#a90000',zorder=12)
ax2.set_zlim(0,2.5)
ax2.set_box_aspect((6.95,9,2.5))
ax2.view_init(65,-70)
ax2.set_title('二维地图 + Nav2 实际路径 · 2.86 m',fontproperties=font,color='white',pad=20,fontsize=17)
for a in (ax,ax2):
    a.set(xlabel='X / m',ylabel='Y / m',zlabel='Z / m')
    a.tick_params(colors='#c9d3df',labelsize=8)
    for axis in (a.xaxis,a.yaxis,a.zaxis):
        axis.label.set_color('#c9d3df')
        axis.set_pane_color((.06,.08,.11,1))
fig.text(.5,.045,'真实保存数据的离线展示；绿色曲线为规划路径，未叠加实时扫描或模拟机器人运动。',
         ha='center',fontproperties=font,color='white',fontsize=13)
fig.subplots_adjust(left=.01,right=.99,bottom=.1,top=.93,wspace=.01)
fig.savefig(out/'dog_navigation_scene.png',dpi=160,facecolor=fig.get_facecolor())
print(out/'dog_navigation_scene.png')
