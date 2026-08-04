#!/usr/bin/env python3
import json
import math
from pathlib import Path

import numpy as np
import yaml
from PIL import Image


base = Path('/home/hoodinga/Documents/SLAM/maps/ackermann_eval_20260804')
meta = yaml.safe_load(base.with_suffix('.yaml').read_text(encoding='utf-8'))
image = np.asarray(Image.open(base.with_suffix('.pgm')))
height, width = image.shape
resolution = float(meta['resolution'])
origin_x, origin_y, _ = [float(v) for v in meta['origin']]
rows, cols = np.nonzero(image < 65)
x = origin_x + (cols + 0.5) * resolution
y = origin_y + (height - 1 - rows + 0.5) * resolution


def median_line(mask, axis):
    values = x[mask] if axis == 'x' else y[mask]
    if len(values) < 5:
        raise RuntimeError(f'insufficient cells for {axis} line: {len(values)}')
    return float(np.median(values)), int(len(values))


west_x, west_n = median_line((x < -5.70) & (y > -3.5) & (y < 4.8), 'x')
east_x, east_n = median_line((x > 5.70) & (y > -2.5) & (y < 4.8), 'x')
north_y, north_n = median_line((y > 5.70) & (x > -4.8) & (x < 4.8), 'y')
south_y, south_n = median_line((y < -4.70) & (x > -4.8) & (x < 1.2), 'y')

# Long inner wall: project occupied cells on its known SDF local axes and use
# the observed face parallel to local X. The wall pose is (1.0, 1.5, yaw=0.25)
# with size 4.2 x 0.18 m; the face toward the start pose is at local n=-0.09.
yaw = 0.25
ux, uy = math.cos(yaw), math.sin(yaw)
nx, ny = -math.sin(yaw), math.cos(yaw)
dx = x - 1.0
dy = y - 1.5
u = dx * ux + dy * uy
n = dx * nx + dy * ny
long_mask = (np.abs(n + 0.09) <= 0.075) & (u >= -2.25) & (u <= 2.25)
long_u = u[long_mask]
if len(long_u) < 20:
    raise RuntimeError(f'insufficient long-wall cells: {len(long_u)}')
long_length = float(long_u.max() - long_u.min())


def metric(name, measured, expected, cells):
    error = abs(measured - expected)
    return {
        'name': name,
        'world_m': expected,
        'map_m': measured,
        'abs_error_m': error,
        'rel_error_pct': 100.0 * error / expected,
        'occupied_cells_used': cells,
    }


metrics = [
    metric('west_east_inner_face_gap', east_x - west_x, 11.8, west_n + east_n),
    metric('south_north_inner_face_gap', north_y - south_y, 10.8, south_n + north_n),
    metric('inner_long_wall_face_length', long_length, 4.2, int(len(long_u))),
]
result = {
    'map': str(base),
    'resolution_m_per_cell': resolution,
    'width_cells': width,
    'height_cells': height,
    'origin': [origin_x, origin_y, 0.0],
    'detected_lines': {
        'west_inner_x_m': west_x,
        'east_inner_x_m': east_x,
        'south_inner_y_m': south_y,
        'north_inner_y_m': north_y,
    },
    'metrics': metrics,
    'mean_abs_error_m': float(np.mean([m['abs_error_m'] for m in metrics])),
    'mean_rel_error_pct': float(np.mean([m['rel_error_pct'] for m in metrics])),
}
print(json.dumps(result, indent=2, sort_keys=True))
Path('/tmp/ackermann_map_measurements.json').write_text(
    json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8'
)
