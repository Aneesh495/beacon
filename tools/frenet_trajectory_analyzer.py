#!/usr/bin/env python3
"""
tools/frenet_trajectory_analyzer.py

Analyzes geometric path curvature profiles, Cartesian-to-Frenet projections,
and validates compliance with friction ellipse acceleration boundaries.
"""

import sys
import math
from typing import List, Tuple, Dict, Any

def compute_path_curvatures(points: List[Tuple[float, float]]) -> List[float]:
    n = len(points)
    if n < 3:
        return [0.0] * n
        
    curvatures = [0.0] * n
    for i in range(1, n - 1):
        x1, y1 = points[i-1]
        x2, y2 = points[i]
        x3, y3 = points[i+1]
        
        # Menger curvature via triangle area: kappa = 4 * Area / (d1 * d2 * d3)
        area2 = abs((x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1))
        d1 = math.hypot(x2 - x1, y2 - y1)
        d2 = math.hypot(x3 - x2, y3 - y2)
        d3 = math.hypot(x3 - x1, y3 - y1)
        
        denom = d1 * d2 * d3
        if denom > 1e-6:
            curvatures[i] = (2.0 * area2) / denom
        else:
            curvatures[i] = 0.0
            
    curvatures[0] = curvatures[1]
    curvatures[-1] = curvatures[-2]
    return curvatures

def verify_friction_limits(
    speeds: List[float],
    curvatures: List[float],
    accelerations: List[float],
    mu_friction: float = 0.85
) -> Dict[str, Any]:
    g = 9.80665
    max_allowed_accel = mu_friction * g
    violations = []
    
    for i in range(len(speeds)):
        v = speeds[i]
        kappa = curvatures[i]
        a_long = accelerations[i] if i < len(accelerations) else 0.0
        a_lat = (v * v) * kappa
        
        # Combined friction circle constraint
        a_total = math.sqrt(a_long * a_long + a_lat * a_lat)
        if a_total > max_allowed_accel:
            violations.append({
                "index": i,
                "speed_mps": v,
                "curvature": kappa,
                "a_lat": a_lat,
                "a_long": a_long,
                "a_total": a_total,
                "margin_exceeded": a_total - max_allowed_accel
            })
            
    return {
        "friction_coefficient": mu_friction,
        "max_accel_g": max_allowed_accel,
        "total_points_evaluated": len(speeds),
        "violations_count": len(violations),
        "violations": violations
    }

def main():
    print("Frenet Trajectory Analyzer: Verification Engine")
    # Synthetic test curve (90 degree turn with R = 25m)
    r = 25.0
    pts = []
    speeds = []
    accels = []
    steps = 50
    for i in range(steps):
        theta = (math.pi * 0.5) * (i / float(steps - 1))
        pts.append((r * math.cos(theta), r * math.sin(theta)))
        speeds.append(10.0) # 10 m/s
        accels.append(0.0)
        
    curvatures = compute_path_curvatures(pts)
    res = verify_friction_limits(speeds, curvatures, accels, 0.85)
    print(f"Points: {res['total_points_evaluated']}, Violations: {res['violations_count']}")
    if res["violations_count"] == 0:
        print("PASS: Trajectory strictly complies with vehicle tire friction envelope.")

if __name__ == "__main__":
    main()
