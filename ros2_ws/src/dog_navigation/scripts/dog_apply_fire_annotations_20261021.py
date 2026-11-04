"""Apply the user's five red regions to occupancy DATA; preserve source and marks.

This is a grid-cell edit, not image generation. Blue centerline samples below
are an approximate transcription of the user's drawing, excluding arrowheads.
"""
import hashlib
import json
import math
from pathlib import Path
import shutil
import numpy as np
from PIL import Image
from scipy import ndimage
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

root = Path(__file__).resolve().parents[2]
base = root / 'dog_deployments/navigation_20261021'
source = base / 'dog_fire_grid_20261021_01'
dest = base / 'dog_fire_grid_20261021_manual_v1'
annotation = Path('C:/Users/Administrator/AppData/Local/Temp/codex-clipboard-c188605d-3b44-42ef-94da-33b45c2e7637.png')
dest.mkdir(exist_ok=False)
shutil.copy2(annotation, dest/'user_annotation.png')
meta = yaml.safe_load((source/'map.yaml').read_text())
original_hash = hashlib.sha256((source/'map.pgm').read_bytes()).hexdigest()
original = np.asarray(Image.open(source/'map.pgm')).copy()
annotated = np.asarray(Image.open(annotation).convert('RGB'))
assert annotated.shape == (1034,1309,3)
red = (annotated[:,:,0]>200) & (annotated[:,:,1]<130) & (annotated[:,:,2]<130)
labels, count = ndimage.label(red)
regions = [ndimage.binary_fill_holes(labels==i) for i in range(1,count+1) if np.count_nonzero(labels==i)>50]
assert len(regions) == 5
# Plot axes in the supplied image: x=-5..7, y=4..-6.
left,right,top,bottom = 94.,1126.,62.,922.
def to_world(px,py):
    return [-5+(px-left)*12/(right-left), 4-(py-top)*10/(bottom-top)]
h,w = original.shape
rows,cols = np.indices((h,w))
res = meta['resolution']; ox,oy,yaw = meta['origin']
assert yaw == 0 and (w,h)==(850,438)
wx = ox+(cols+.5)*res
wy = oy+(h-rows-.5)*res
px = np.rint(left+(wx+5)/12*(right-left)).astype(int)
py = np.rint(top+(4-wy)/10*(bottom-top)).astype(int)
visible = (px>=0)&(px<annotated.shape[1])&(py>=0)&(py<annotated.shape[0])
edited = original.copy()
entries = []
for number, region in enumerate(regions,1):
    inside = np.zeros_like(original,dtype=bool)
    inside[visible] = region[py[visible],px[visible]]
    change = inside & (original==0)
    edited[change] = 254
    ys,xs = np.where(region)
    entries.append(dict(region=number,annotation_bbox_px=[int(xs.min()),int(ys.min()),int(xs.max()),int(ys.max())],
                        changed_occupied_cells=int(change.sum()),
                        world_center_m=to_world(float(xs.mean()),float(ys.mean())),
                        changed_cells_pgm_row_col=np.argwhere(change).tolist()))
assert all(e['changed_occupied_cells']>0 for e in entries)
changed = original!=edited
assert np.all(original[changed]==0) and np.all(edited[changed]==254)
assert np.array_equal(edited[original==205],original[original==205])
Image.fromarray(edited).save(dest/'map.pgm')
(dest/'map.yaml').write_text(yaml.safe_dump(meta,sort_keys=False),encoding='utf-8')
route_pixels = [[460,406],[479,400],[504,395],[529,392],[555,389],[581,383],
 [609,376],[645,366],[688,366],[704,370],[724,383],[735,395],[740,410],
 [747,425],[748,447],[748,482],[741,494],[721,504],[697,511],[670,517],
 [644,519],[581,519],[550,511],[526,502],[495,494],[472,487]]
points = np.array([to_world(*p) for p in route_pixels])
def cell(point):
    c=int(math.floor((point[0]-ox)/res)); r=h-1-int(math.floor((point[1]-oy)/res))
    return int(edited[r,c]) if 0<=r<h and 0<=c<w else None
samples=[]
for a,b in zip(points,points[1:]):
    n=max(1,math.ceil(float(np.linalg.norm(b-a))/(res/10)))
    samples.extend(cell(a+(b-a)*i/n) for i in range(n+1))
route=dict(frame_id='dog_lio_map',source='user blue drawing, manually transcribed centerline',
           direction='clockwise',closed=False,
           closure_note='User drawing has an open gap on the left; no return segment invented.',
           purpose='reference route only; not a Nav2 computed path or motor command',
           source_pixels=route_pixels,points_xy_m=points.tolist(),
           length_m=float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum()),
           start_xy_m=points[0].tolist(),end_xy_m=points[-1].tolist(),
           occupancy_audit=dict(sample_spacing_max_m=res/10,samples=len(samples),
                                occupied=sum(v==0 for v in samples),unknown=sum(v==205 for v in samples),
                                out_of_bounds=sum(v is None for v in samples)),
           footprint_or_dynamic_obstacles_checked=False)
(dest/'reference_route.json').write_text(json.dumps(route,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
manifest=dict(source_map=str(source/'map.yaml'),source_pgm_sha256=original_hash,
              annotation_sha256=hashlib.sha256(annotation.read_bytes()).hexdigest(),
              edited_pgm_sha256=hashlib.sha256((dest/'map.pgm').read_bytes()).hexdigest(),
              screenshot_registration=dict(pixel_axes=[left,right,top,bottom],world_bounds=[-5,7,-6,4]),
              operation='Only occupied cells inside the five filled red outlines become free. Unknown cells preserved.',
              changed_cells=int(changed.sum()),regions=entries,original_preserved=True)
(dest/'edit_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
assert hashlib.sha256((source/'map.pgm').read_bytes()).hexdigest()==original_hash
font=FontProperties(fname='C:/Windows/Fonts/msyh.ttc')
fig,axs=plt.subplots(1,2,figsize=(16,8.5))
for ax,grid,title in zip(axs,[original,edited],['修改前 · 红圈范围','修改后 · 蓝线参考路线']):
    ax.imshow(np.flipud(grid),origin='lower',extent=[ox,ox+w*res,oy,oy+h*res],cmap='gray',vmin=0,vmax=254,interpolation='nearest')
    ax.set(xlim=(-5,7),ylim=(-6,4),xlabel='X / m',ylabel='Y / m')
    ax.set_aspect('equal'); ax.set_title(title,fontproperties=font,fontsize=16)
for idx,region in enumerate(regions,1):
    ys,xs=np.where(region)
    cx,cy=to_world(float(xs.mean()),float(ys.mean()))
    # Exact user mask contour mapped to world, without drawing on map data.
    crop=region[ys.min():ys.max()+1,xs.min():xs.max()+1]
    xx=np.array([to_world(v,0)[0] for v in range(xs.min(),xs.max()+1)])
    yy=np.array([to_world(0,v)[1] for v in range(ys.min(),ys.max()+1)])
    axs[0].contour(xx,yy,crop.astype(float),levels=[.5],colors=['#ef4444'],linewidths=1.6)
    axs[0].text(cx,cy+.35,str(idx),color='#c62020',ha='center',fontsize=10)
axs[1].plot(*points.T,color='#079cfa',lw=2.5)
for i in (4,13,21):
    axs[1].annotate('',xy=points[i+1],xytext=points[i],arrowprops=dict(arrowstyle='->',color='#079cfa',lw=2.5))
axs[1].scatter(*points[0],c='#069d67',s=45,zorder=4)
axs[1].scatter(*points[-1],c='#079cfa',s=45,zorder=4)
fig.suptitle(f'按用户标注修改 · 五处障碍共清除 {int(changed.sum())} 格 · 原图保留',fontproperties=font,fontsize=19)
fig.text(.5,.025,'蓝线为顺时针参考路线；保留左侧缺口，未自动闭合，未发送运动指令。',ha='center',fontproperties=font,fontsize=12)
fig.tight_layout(rect=[0,.06,1,.94]); fig.savefig(dest/'edit_comparison.png',dpi=150)
print(json.dumps(dict(output=str(dest),changed_cells=int(changed.sum()),
                     per_region=[e['changed_occupied_cells'] for e in entries],route=route),ensure_ascii=False))
