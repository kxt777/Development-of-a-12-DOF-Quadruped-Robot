"""Inspect an uncompressed binary XYZI PCD without declaring it traversable."""
import argparse
import json
from pathlib import Path

import numpy as np


def read_pcd(path):
    with Path(path).open('rb') as f:
        header = {}
        while True:
            line = f.readline().decode('ascii').strip()
            if not line:
                raise ValueError('Incomplete PCD header')
            if line.startswith('#'):
                continue
            key, *values = line.split()
            header[key] = values
            if key == 'DATA':
                break
        if header['DATA'] != ['binary']:
            raise ValueError('Only uncompressed binary PCD supported')
        types = {('F', '4'): '<f4', ('F', '8'): '<f8',
                 ('U', '1'): 'u1', ('U', '2'): '<u2', ('U', '4'): '<u4',
                 ('I', '1'): 'i1', ('I', '2'): '<i2', ('I', '4'): '<i4'}
        dtype = np.dtype([(name, types[(kind, size)], (int(count),))
                          for name, kind, size, count in zip(
                              header['FIELDS'], header['TYPE'], header['SIZE'], header['COUNT'])])
        data = np.frombuffer(f.read(), dtype=dtype)
        if len(data) != int(header['POINTS'][0]):
            raise ValueError('PCD payload length does not match header')
    return np.column_stack([data[k][:, 0] for k in ('x', 'y', 'z')])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('pcd', type=Path)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists() or args.output.with_suffix('.json').exists():
        raise FileExistsError('Use a new preview output path')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    xyz = read_pcd(args.pcd)
    finite = np.isfinite(xyz).all(axis=1)
    pts = xyz[finite]
    if not len(pts):
        raise ValueError('No finite points')
    stats = {'source': str(args.pcd), 'total_points': len(xyz),
             'finite_points': int(finite.sum()), 'frame': 'dog_lio_map',
             'xyz_min': pts.min(axis=0).tolist(), 'xyz_max': pts.max(axis=0).tolist(),
             'xyz_percentiles_1_50_99': np.percentile(pts, [1, 50, 99], axis=0).tolist(),
             'note': 'Measured point cloud preview, not a navigable occupancy map. Z is in the initial LIO frame, not height above floor.'}
    draw = pts[::max(1, len(pts)//60000)]
    fig, axes = plt.subplots(1, 2, figsize=(13, 6), constrained_layout=True)
    for ax, dims, labels in [(axes[0], (0, 1), ('X (m)', 'Y (m)')),
                              (axes[1], (0, 2), ('X (m)', 'Z (m)'))]:
        im = ax.scatter(draw[:, dims[0]], draw[:, dims[1]], c=draw[:, 2],
                        s=.6, cmap='viridis', rasterized=True)
        ax.set(xlabel=labels[0], ylabel=labels[1])
        ax.set_aspect('equal', adjustable='datalim')
        ax.grid(alpha=.2)
        ax.scatter([0], [0], marker='+', c='red', s=60, label='Initial IMU origin')
        ax.legend(loc='upper right')
    axes[0].set_title('Top view: measured returns')
    axes[1].set_title('Side view: measured returns')
    fig.colorbar(im, ax=axes, label='Z in initial LIO frame (m)', shrink=.7)
    fig.suptitle('First live FAST-LIO map | %s points | no motion command sent' % len(pts))
    fig.savefig(args.output, dpi=160)
    args.output.with_suffix('.json').write_text(json.dumps(stats, indent=2), encoding='utf-8')
    print(json.dumps(stats, indent=2))


if __name__ == '__main__':
    main()
