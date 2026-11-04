"""Create offline 3D previews from a real PCD; rendering never modifies the map."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from map_preview import read_pcd


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('pcd', type=Path)
    p.add_argument('--output-stem', type=Path, required=True)
    p.add_argument('--capture-seconds', type=float, default=60)
    args = p.parse_args()
    stem = args.output_stem
    if any(stem.with_suffix(ext).exists() for ext in ('.html', '.png', '.json')):
        raise FileExistsError('Use a new output stem')
    xyz = read_pcd(args.pcd)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    draw = xyz[::max(1, int(np.ceil(len(xyz) / 140000)))]
    # This slice is only a viewer option, never a ground/obstacle classification.
    cut_z = 1.5
    low = draw[draw[:, 2] <= cut_z]
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / '.preview_deps'))
    import plotly.graph_objects as go
    fig = go.Figure()
    for pts, name, visible in [(draw, '完整点云', True), (low, '下部结构显示切片', False)]:
        fig.add_trace(go.Scatter3d(x=pts[:, 0], y=pts[:, 1], z=pts[:, 2],
                                  mode='markers', name=name, visible=visible,
                                  marker=dict(size=1.2, color=pts[:, 2], colorscale='Turbo',
                                              cmin=float(draw[:, 2].min()), cmax=float(draw[:, 2].max()),
                                              opacity=1, colorbar=dict(title='Z / m')),
                                  hoverinfo='skip'))
    fig.update_layout(template='plotly_dark', margin=dict(l=0, r=0, t=65, b=0),
                      scene=dict(aspectmode='data', xaxis_title='X / m', yaxis_title='Y / m',
                                 zaxis_title='Z / m', camera=dict(eye=dict(x=1.3, y=1.3, z=.8))),
                      updatemenus=[dict(type='buttons', direction='right', x=.01, y=1.08,
                                        buttons=[dict(label='完整点云', method='update', args=[{'visible': [True, False]}]),
                                                 dict(label='查看下部结构（Z ≤ 1.5 m）', method='update', args=[{'visible': [False, True]}])])],
                      showlegend=False)
    chart = fig.to_html(full_html=False, include_plotlyjs=True,
                        config={'displaylogo': False, 'responsive': True, 'scrollZoom': True},
                        default_height='calc(100vh - 140px)')
    html = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>机械狗 · FAST-LIO 实测三维地图</title>
<style>body{margin:0;background:#111827;color:#e5e7eb;font:15px system-ui,sans-serif}header{padding:18px 24px}h1{font-size:22px;margin:0 0 8px}p{margin:4px 0;color:#aebbd0}</style>
<header><h1>FAST-LIO · 实测三维点云地图</h1>
<p>%s 秒采集 · %s 个实测点 · 拖动旋转，滚轮缩放，右键拖动平移</p>
<p>颜色表示初始建图坐标中的 Z 值，不代表物体类别。显示切片只用于看清遮挡，完整 PCD 保持原样。当前为局部地图。</p></header>''' % (format(args.capture_seconds, '.3g'), len(xyz))
    stem.with_suffix('.html').write_text(html + chart + '</html>', encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig2 = plt.figure(figsize=(14, 7), facecolor='#101721')
    for index, pts, title in [(1, draw, 'Complete measured cloud'),
                              (2, low, 'Display slice: Z <= 1.5 m (PCD unchanged)')]:
        ax = fig2.add_subplot(1, 2, index, projection='3d', facecolor='#101721')
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=pts[:, 2], cmap='turbo',
                   s=.45, linewidths=0, depthshade=False, vmin=draw[:, 2].min(), vmax=draw[:, 2].max())
        ax.set(xlabel='X / m', ylabel='Y / m', zlabel='Z / m')
        ax.set_box_aspect(np.maximum(np.ptp(pts, axis=0), .1))
        ax.view_init(elev=30, azim=45)
        ax.set_title(title, color='white', pad=15)
        ax.tick_params(colors='#afbdce', labelsize=8)
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.label.set_color('#afbdce')
            axis.set_pane_color((.06, .09, .13, 1))
    fig2.suptitle('FAST-LIO | real MID-360 capture | %s points | %.3g seconds' % (len(xyz), args.capture_seconds), color='white', fontsize=16)
    fig2.subplots_adjust(left=0, right=.98, bottom=.03, top=.88, wspace=.02)
    fig2.savefig(stem.with_suffix('.png'), dpi=160, facecolor=fig2.get_facecolor())
    meta = {'pcd': str(args.pcd.resolve()), 'points': len(xyz), 'display_points': len(draw),
            'capture_seconds': args.capture_seconds,
            'display_slice_max_z_m': cut_z, 'slice_changes_saved_map': False,
            'sha256': hashlib.sha256(args.pcd.read_bytes()).hexdigest()}
    stem.with_suffix('.json').write_text(json.dumps(meta, indent=2), encoding='utf-8')
    print(json.dumps(meta, indent=2))


if __name__ == '__main__':
    main()
