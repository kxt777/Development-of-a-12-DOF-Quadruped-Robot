"""Build a compact, auditable lidar map with an explicitly unregistered semantic layer."""
import base64
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import zipfile
import numpy as np
from scipy.optimize import linear_sum_assignment
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path('D:/Project/dog/demov3')
SRC=ROOT/'20261027_201820'
OUT=ROOT/'semantic_map_v1_20261028'
OUT.mkdir(exist_ok=True)
(OUT/'keyframes').mkdir(exist_ok=True)

def compact_json(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,separators=(',',':'),allow_nan=False),encoding='utf-8')

def load_pcd(path):
    with path.open('rb') as stream:
        header=[]
        while True:
            line=stream.readline().decode('ascii').strip()
            header.append(line)
            if line.startswith('DATA '):break
        assert 'DATA binary' in header
        assert 'FIELDS x y z intensity normal_x normal_y normal_z curvature' in header
        raw=np.frombuffer(stream.read(),dtype='<f4').reshape(-1,8)
        return raw.copy()

def voxelize(raw,size):
    indices=np.floor(raw[:,:3]/size).astype(np.int32)
    cells,reverse,count=np.unique(indices,axis=0,return_inverse=True,return_counts=True)
    centroids=np.column_stack([np.bincount(reverse,weights=raw[:,i])/count for i in range(4)]).astype('<f4')
    return cells,centroids,count.astype(np.uint32)

raw=load_pcd(SRC/'map.pcd')
valid=np.isfinite(raw[:,:4]).all(axis=1)
data=raw[valid]
cells,points,counts=voxelize(data,.05)
assert counts.sum()==len(data)
assert np.all(np.floor(points[:,:3]/.05).astype(np.int32)==cells)
np.savez_compressed(OUT/'voxel_map_5cm.npz',voxel_indices=cells,centroids_xyz=points[:,:3],intensity=points[:,3],sample_count=counts,voxel_size_m=np.float64(.05),frame_id=np.array('dog_lio_map'),semantic_class_id=np.full(len(cells),-1,np.int8))
header=f'''# .PCD v0.7 - voxel centroids, 0.05 m; no semantic positions assigned
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {len(points)}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {len(points)}
DATA binary
'''
with (OUT/'map_5cm.pcd').open('wb') as f:
    f.write(header.encode()); f.write(points.tobytes())
pcd=(OUT/'map_5cm.pcd').read_bytes()
assert len(pcd.split(b'DATA binary\n',1)[1])==len(points)*16
npz=np.load(OUT/'voxel_map_5cm.npz')
assert np.array_equal(npz['sample_count'],counts)
assert np.all(npz['semantic_class_id']==-1)

rows=[json.loads(line) for line in (SRC/'semantic_results.jsonl').read_text().splitlines()]
epoch=1793103504.655180698

def iou(a,b):
    intersection=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
    union=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-intersection
    return intersection/union if union>0 else 0.

tracks=[]
for row in rows:
    for label in ['fire','chemical','human']:
        objects=[o for o in row['objects'] if o['label']==label]
        active=[t for t in tracks if t['label']==label and 0<=row['stamp']-t['last_stamp']<=.5]
        matched=set()
        if active and objects:
            costs=np.array([[1-iou(t['observations'][-1]['box'],o['bbox_xyxy']) for o in objects] for t in active])
            for ti,oi in zip(*linear_sum_assignment(costs)):
                if costs[ti,oi]>.8:continue
                active[ti]['observations'].append({'stamp':row['stamp'],'score':objects[oi]['score'],'box':objects[oi]['bbox_xyxy'],'camera_xyz_m':objects[oi]['xyz_m']})
                active[ti]['last_stamp']=row['stamp']
                matched.add(oi)
        for oi,obj in enumerate(objects):
            if oi in matched:continue
            tracks.append({'id':f'obs_{len(tracks)+1:04d}','label':label,'last_stamp':row['stamp'],'observations':[{'stamp':row['stamp'],'score':obj['score'],'box':obj['bbox_xyxy'],'camera_xyz_m':obj['xyz_m']}]})

events=[]
for track in tracks:
    obs=track['observations']
    best=max(obs,key=lambda o:o['score'])
    scores=np.array([o['score'] for o in obs])
    xyz=[o['camera_xyz_m'] for o in obs if o['camera_xyz_m'] is not None]
    stable=bool(len(obs)>=3 and sum(scores>=.6)>=3 and scores.max()>=.7 and obs[-1]['stamp']-obs[0]['stamp']>=.19)
    z=[v[2] for v in xyz]
    event={'id':track['id'],'label':track['label'],'start_s':round(obs[0]['stamp']-epoch,3),'end_s':round(obs[-1]['stamp']-epoch,3),'frames':len(obs),'score_max':round(float(scores.max()),3),'score_median':round(float(np.median(scores)),3),'valid_depth_frames':len(xyz),'camera_depth_median_m':round(float(np.median(z)),3) if z else None,'camera_depth_p10_p90_m':np.round(np.percentile(z,[10,90]),3).tolist() if z else None,'candidate_for_review':stable,'review_status':'unreviewed_model_prediction','position_status':'unregistered','map_xyz_m':None,'representative':{'stamp':best['stamp'],'time_s':round(best['stamp']-epoch,3),'bbox_xyxy':np.round(best['box'],1).tolist(),'camera_xyz_m':np.round(best['camera_xyz_m'],3).tolist() if best['camera_xyz_m'] is not None else None,'image':None},'observations':[[round(o['stamp']-epoch,3),round(o['score'],3),np.round(o['box'],1).tolist(),np.round(o['camera_xyz_m'],3).tolist() if o['camera_xyz_m'] is not None else None] for o in obs]}
    events.append(event)
assert sum(e['frames'] for e in events)==sum(len(r['objects']) for r in rows)
compact_json(OUT/'semantic_observations.json',{'schema':'dog.semantic_observations.v1','source_frame':'dog_oak_rgb_camera_optical_frame','map_frame':'dog_lio_map','registration_status':'blocked_missing_trajectory_and_extrinsics','observations_are_unique_physical_objects':False,'relative_time_epoch_unix_s':epoch,'events':events})

selected=[e for e in events if e['candidate_for_review']]
# Keep a bounded number of readable evidence images; every numerical observation is retained.
keyframes=[]
for label in ['fire','chemical','human']:
    pool=sorted([e for e in events if e['label']==label],key=lambda e:(e['candidate_for_review'],e['score_max']),reverse=True)
    keyframes.extend(pool[:8])
compact_json(OUT/'keyframe_requests.json',[{'id':e['id'],'label':e['label'],**e['representative']} for e in keyframes])

# A projection of observed returns only. Empty pixels are unknown, never asserted free.
xy=cells[:,:2]
lo=xy.min(axis=0); hi=xy.max(axis=0)
density=np.zeros((hi[1]-lo[1]+1,hi[0]-lo[0]+1),dtype=np.uint32)
np.add.at(density,(xy[:,1]-lo[1],xy[:,0]-lo[0]),counts)
extent=[lo[0]*.05,(hi[0]+1)*.05,lo[1]*.05,(hi[1]+1)*.05]
fig,ax=plt.subplots(figsize=(12,8),dpi=160)
masked=np.ma.masked_where(density==0,np.log1p(density))
im=ax.imshow(masked,origin='lower',extent=extent,cmap='viridis',interpolation='nearest',aspect='equal')
ax.set(xlabel='X in dog_lio_map (m)',ylabel='Y in dog_lio_map (m)',title='demov3 | LiDAR return-density projection | 5 cm cells')
fig.colorbar(im,ax=ax,label='log(1 + return count)')
ax.text(.01,.01,'Blank = unknown; colors = return density, not semantic class.\nSemantic observations are not spatially registered.',transform=ax.transAxes,fontsize=9,bbox=dict(facecolor='white',alpha=.9,edgecolor='none'))
fig.tight_layout();fig.savefig(OUT/'map_top_view.png');plt.close(fig)
np.savez_compressed(OUT/'xy_return_density_5cm.npz',return_count=density,origin_xy_m=lo*.05,resolution_m=.05,empty_means=np.array('unknown_not_free'))

_,lod,lod_counts=voxelize(data,.1)
# Display-only coordinates are rounded to centimetres. Full 5 cm centroids remain in the map files.
display={'xyz_i16_cm':base64.b64encode(np.rint(lod[:,:3]*100).astype('<i2').tobytes()).decode(),'point_count':len(lod),'bounds_min':data[:,:3].min(axis=0).tolist(),'bounds_max':data[:,:3].max(axis=0).tolist(),'focus_min':np.percentile(data[:,:3],1,axis=0).tolist(),'focus_max':np.percentile(data[:,:3],99,axis=0).tolist()}
compact_json(OUT/'viewer_data.json',display)
summary={'schema':'dog.semantic_map_bundle.v1','status':'geometry_ready_semantics_unregistered','source':'demov3/20261027_201820','map_frame':'dog_lio_map','original_points':len(raw),'valid_points':len(data),'nonfinite_removed':int((~valid).sum()),'voxel_size_m':.05,'compressed_points':len(points),'point_reduction_percent':round(100*(1-len(points)/len(raw)),2),'original_pcd_bytes':(SRC/'map.pcd').stat().st_size,'compact_voxel_npz_bytes':(OUT/'voxel_map_5cm.npz').stat().st_size,'pcd_bytes':(OUT/'map_5cm.pcd').stat().st_size,'original_observation_count':sum(len(r['objects']) for r in rows),'observation_segments':len(events),'candidate_segments_for_review':len(selected),'counts_by_class':dict(Counter(e['label'] for e in events)),'candidate_segments_by_class':dict(Counter(e['label'] for e in selected)),'unique_object_count':None,'spatially_registered_objects':0,'missing':['timestamped T_map_lidar trajectory or TF','calibrated T_lidar_camera extrinsic transform'],'candidate_rule':'>=3 observations with confidence>=0.60, maximum>=0.70, duration>=0.19s; review priority only, not ground truth','compression_method':'Unweighted per-return centroid and mean intensity per 5cm voxel; sample count retained. No ROI/outlier removal or semantic fabrication.','semantic_class_id_minus_one':'Unassigned: no camera-to-map transform available','source_hashes':{name:hashlib.sha256((SRC/name).read_bytes()).hexdigest() for name in ['map.pcd','semantic_results.jsonl']}}
(OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(summary,ensure_ascii=False),flush=True)
