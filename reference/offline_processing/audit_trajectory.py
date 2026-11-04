import json
from pathlib import Path
import sqlite3
import struct
import numpy as np

root=Path('D:/Project/dog/demov3')
recovered=root/'semantic_map_work/recovered'
times=[]
for p in sorted((root/'20261027_201820/rosbag').glob('*.db3')):
    c=sqlite3.connect(str(p))
    tid=c.execute("SELECT id FROM topics WHERE name='/dog_mapping/cloud_registered'").fetchone()[0]
    for t,d in c.execute('SELECT timestamp,substr(data,1,24) FROM messages WHERE topic_id=? ORDER BY timestamp',(tid,)):
        times.append((t/1e9,struct.unpack_from('<I',d,4)[0]+struct.unpack_from('<I',d,8)[0]/1e9))
times=np.array(times)
np.save(recovered/'cloud_timestamps.npy',times)
pose=np.loadtxt(recovered/'mat_pre.txt')
print('log shape',pose.shape,'time range',pose[0,0],pose[-1,0])
print('clouds',times.shape,'first',times[:3].tolist(),'last',times[-3:].tolist())
print('pose xyz bounds',pose[:,4:7].min(axis=0),pose[:,4:7].max(axis=0))
print('pose time steps',np.percentile(np.diff(pose[:,0]),[0,1,50,99,100]))
print('cloud time steps',np.percentile(np.diff(times[:,1]),[0,1,50,99,100]))
if len(pose)==len(times):
    offset=times[:,1]-pose[:,0]
    print('same-index offset distribution',np.percentile(offset,[0,1,50,99,100]))
np.savetxt(recovered/'prediction_log_preview.csv',pose[:10],delimiter=',')
