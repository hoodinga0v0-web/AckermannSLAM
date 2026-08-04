#!/usr/bin/env python3
import json
import math
from pathlib import Path

import numpy as np


root = Path('/tmp/ackermann_quant_results')


def load(prefix):
    return [json.loads((root / f'{prefix}_{i}.json').read_text()) for i in range(1, 6)]


def summary(values):
    array = np.asarray(values, dtype=float)
    return {
        'mean': float(array.mean()),
        'sample_std': float(array.std(ddof=1)),
        'min': float(array.min()),
        'max': float(array.max()),
    }


straight = load('straight')
left = load('arc_left')
right = load('arc_right')

straight_summary = {
    'n': 5,
    'gt_distance_m': summary([r['gt_distance_m'] for r in straight]),
    'odom_distance_m': summary([r['odom_distance_m'] for r in straight]),
    'cmd_integral_distance_m': summary([r['cmd_linear_integral_m'] for r in straight]),
    'odom_gt_abs_error_m': summary([r['odom_gt_abs_error_m'] for r in straight]),
    'odom_gt_rel_error_pct': summary([r['odom_gt_rel_error_pct'] for r in straight]),
    'gt_lateral_abs_m': summary([abs(r['gt_lateral_m']) for r in straight]),
}


def arc_summary(rows, direction):
    gt = [r['gt_circle_fit']['radius_m'] for r in rows]
    odom = [r['odom_circle_fit']['radius_m'] for r in rows]
    reference = [r['cmd_reference_radius_m'] for r in rows]
    gt_error = [abs(a - b) for a, b in zip(gt, reference)]
    gt_error_pct = [100.0 * e / b for e, b in zip(gt_error, reference)]
    inner_key = 'front_left' if direction == 'left' else 'front_right'
    outer_key = 'front_right' if direction == 'left' else 'front_left'
    inner = [abs(r['steering_mean_rad'][inner_key]) for r in rows]
    outer = [abs(r['steering_mean_rad'][outer_key]) for r in rows]
    return {
        'n': 5,
        'cmd_reference_radius_m': summary(reference),
        'gt_radius_m': summary(gt),
        'gt_radius_abs_error_m': summary(gt_error),
        'gt_radius_rel_error_pct': summary(gt_error_pct),
        'odom_radius_m': summary(odom),
        'circle_fit_residual_rms_m': summary([r['gt_circle_fit']['residual_rms_m'] for r in rows]),
        'inner_steering_rad': summary(inner),
        'outer_steering_rad': summary(outer),
        'inner_steering_deg': summary([math.degrees(v) for v in inner]),
        'outer_steering_deg': summary([math.degrees(v) for v in outer]),
    }


left_summary = arc_summary(left, 'left')
right_summary = arc_summary(right, 'right')

fusion_theory = []
wheelbase = 171.0
track = 130.0
for inner_deg in (10.0, 20.0, 30.0, 40.0):
    inner = math.radians(inner_deg)
    outer = math.atan(1.0 / (1.0 / math.tan(inner) + track / wheelbase))
    fusion_theory.append({'inner_deg': inner_deg, 'ideal_outer_deg': math.degrees(outer)})

result = {
    'straight': straight_summary,
    'arc_left': left_summary,
    'arc_right': right_summary,
    'arc_left_right_mean_radius_symmetry_error_m': abs(
        left_summary['gt_radius_m']['mean'] - right_summary['gt_radius_m']['mean']
    ),
    'arc_left_right_mean_radius_symmetry_error_pct_of_0_5m': 200.0 * abs(
        left_summary['gt_radius_m']['mean'] - right_summary['gt_radius_m']['mean']
    ),
    'fusion_ideal_only': fusion_theory,
}

Path('/tmp/ackermann_quant_summary.json').write_text(
    json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8'
)
print(json.dumps(result, indent=2, sort_keys=True))
