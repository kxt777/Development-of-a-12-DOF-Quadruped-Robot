"""Plot existing occupancy data without changing any cell or map metadata."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Patch

p = argparse.ArgumentParser()
p.add_argument('map_yaml', type=Path)
p.add_argument('output', type=Path)
p.add_argument('--bounds', nargs=4, type=float, metavar=('XMIN','XMAX','YMIN','YMAX'))
args = p.parse_args()
meta = yaml.safe_load(args.map_yaml.read_text())
source = args.map_yaml.parent / meta['image']
before = hashlib.sha256(source.read_bytes()).hexdigest()
data = np.asarray(Image.open(source))
h,w = data.shape
x,y,yaw = meta['origin']
assert yaw == 0
r = meta['resolution']
font = FontProperties(fname='C:/Windows/Fonts/msyh.ttc')
fig, ax = plt.subplots(figsize=(15,9))
ax.imshow(np.flipud(data), origin='lower', cmap='gray', vmin=0, vmax=254,
          interpolation='nearest', extent=[x,x+w*r,y,y+h*r])
if args.bounds:
    ax.set_xlim(args.bounds[:2]); ax.set_ylim(args.bounds[2:])
ax.set_aspect('equal')
ax.set_xlabel('X / m'); ax.set_ylabel('Y / m')
ax.set_title('模拟火场 · 二维占据栅格' + ('（局部放大）' if args.bounds else '（完整范围）') +
             f'\n5 厘米/格 · {w} × {h} 格 · 未擦除噪声', fontproperties=font, fontsize=19)
ax.legend(handles=[Patch(facecolor='black',label='障碍'),Patch(facecolor='white',edgecolor='#aaa',label='空闲'),
                   Patch(facecolor=str(205/254),label='未知')], prop=font, loc='upper right')
fig.tight_layout()
fig.savefig(args.output,dpi=160)
assert hashlib.sha256(source.read_bytes()).hexdigest() == before
print(json.dumps({'size':[w,h], 'resolution':r, 'origin':meta['origin'],
                  'cells':{str(int(k)):int(v) for k,v in zip(*np.unique(data,return_counts=True))},
                  'source_sha256':before, 'preview':str(args.output)},ensure_ascii=False))
