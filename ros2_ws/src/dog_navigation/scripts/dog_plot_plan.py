"""Render a saved Nav2 result over the original occupancy image."""
import argparse
import json
from pathlib import Path
import numpy as np
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Patch
import yaml

parser = argparse.ArgumentParser()
parser.add_argument('map_yaml', type=Path)
parser.add_argument('path_json', type=Path)
parser.add_argument('output', type=Path)
args = parser.parse_args()
meta = yaml.safe_load(args.map_yaml.read_text())
image = np.asarray(Image.open(args.map_yaml.parent / meta['image']))
data = json.loads(args.path_json.read_text())
points = np.asarray(data['points'])
ox, oy, yaw = meta['origin']
assert yaw == 0, 'This preview expects an axis-aligned saved grid'
r = meta['resolution']
font = FontProperties(fname='C:/Windows/Fonts/msyh.ttc')
fig, ax = plt.subplots(figsize=(10, 10))
ax.imshow(np.flipud(image), origin='lower', cmap='gray', vmin=0, vmax=254,
          interpolation='nearest', extent=[ox, ox+image.shape[1]*r, oy, oy+image.shape[0]*r])
ax.plot(*np.array([data['start'], data['goal']]).T, '--', color='#d88a00', lw=1.5, label='Start-goal straight line')
ax.plot(*points.T, color='#007bd9', lw=3, label='Nav2 / NavFn path')
ax.scatter(*data['start'], s=110, c='#00a66c', edgecolors='white', zorder=5)
ax.scatter(*data['goal'], marker='*', s=280, c='#db315a', edgecolors='white', zorder=5)
ax.annotate('起点', data['start'], xytext=(-38, -25), textcoords='offset points', fontproperties=font, fontsize=14)
ax.annotate('终点', data['goal'], xytext=(15, -25), textcoords='offset points', fontproperties=font, fontsize=14)
ax.set_xlim(-1.9, 5.05)
ax.set_ylim(-2.5, 3.55)
ax.set_aspect('equal')
ax.set_xlabel('X / m')
ax.set_ylabel('Y / m')
ax.set_title(f"Nav2 实际规划结果：{data['path_length_m']:.2f} 米\n原始占据地图 · 无去噪 · 无膨胀 · 点机器人", fontproperties=font, fontsize=18)
ax.legend(loc='lower right')
fig.text(.5, .04, '黑色：障碍物　白色：空闲　灰色：未知\n测试起终点；此图证明规划链路，不代表狗已执行路线。',
         ha='center', fontproperties=font, fontsize=12)
fig.tight_layout(rect=(0,.08,1,1))
fig.savefig(args.output, dpi=150)
print(args.output)
