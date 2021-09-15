#!/usr/bin/env python3
"""
tools/scenario_synthesizer.py

Procedural scenario generator for Beacon digital twin simulations.
Synthesizes complex multi-agent road environments (roundabouts, highway merges,
urban grids) into declarative JSON configurations.
"""

import sys
import json
import math
import random
from typing import Dict, Any, List

def generate_roundabout_scenario(
    num_vehicles: int = 8,
    radius: float = 25.0,
    speed_limit: float = 8.0,
    duration: float = 30.0
) -> Dict[str, Any]:
    vehicles = []
    
    for i in range(num_vehicles):
        angle = (2.0 * math.pi * i) / num_vehicles
        # Approaches from external spoke
        approach_dist = radius + 30.0
        spawn_x = approach_dist * math.cos(angle)
        spawn_z = approach_dist * math.sin(angle)
        heading = angle + math.pi # Heading inward toward roundabout
        
        # Target destination: opposite spoke
        dest_angle = angle + math.pi
        dest_x = (approach_dist + 10.0) * math.cos(dest_angle)
        dest_z = (approach_dist + 10.0) * math.sin(dest_angle)
        
        vehicles.append({
            "id": f"roundabout_veh_{i:02d}",
            "spawn": [round(spawn_x, 2), 0.0, round(spawn_z, 2)],
            "heading": round(heading, 4),
            "destination": [round(dest_x, 2), round(dest_z, 2)],
            "speed_limit": speed_limit
        })
        
    return {
        "scenario_id": f"procedural_roundabout_{num_vehicles}v",
        "name": f"Procedural {num_vehicles}-Vehicle Roundabout Coordination",
        "description": "Multi-vehicle circulation and gap acceptance inside a rotary conflict zone using V2X reservations.",
        "duration_sec": duration,
        "intersection": {
            "center": [0.0, 0.0],
            "conflict_radius": radius + 4.0
        },
        "vehicles": vehicles
    }

def generate_highway_merge_scenario(
    mainline_count: int = 6,
    ramp_count: int = 4,
    speed_limit: float = 24.0,
    duration: float = 25.0
) -> Dict[str, Any]:
    vehicles = []
    
    # Mainline vehicles along X axis (Z = 0)
    for i in range(mainline_count):
        spawn_x = -float(i * 35)
        vehicles.append({
            "id": f"mainline_{i:02d}",
            "spawn": [spawn_x, 0.0, 0.0],
            "heading": 0.0,
            "destination": [350.0, 0.0],
            "speed_limit": speed_limit
        })
        
    # On-ramp vehicles approaching at an angle from Z = 20 to merge at X = 100
    for j in range(ramp_count):
        spawn_x = -float(j * 30 + 50)
        vehicles.append({
            "id": f"ramp_{j:02d}",
            "spawn": [spawn_x, 0.0, 15.0],
            "heading": -0.15,
            "destination": [350.0, 0.0],
            "speed_limit": speed_limit * 0.85
        })
        
    return {
        "scenario_id": f"procedural_merge_{mainline_count + ramp_count}v",
        "name": "Highway On-Ramp Cooperative Zipper Merge",
        "description": "Mainline and ramp vehicles executing cooperative speed adjustments for seamless zipper merging.",
        "duration_sec": duration,
        "vehicles": vehicles
    }

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 tools/scenario_synthesizer.py <roundabout|merge> <output.json> [vehicle_count]")
        sys.exit(0)
        
    scenario_type = sys.argv[1].lower()
    output_path = sys.argv[2]
    count = int(sys.argv[3]) if len(sys.argv) > 3 else 8
    
    if scenario_type == "roundabout":
        data = generate_roundabout_scenario(num_vehicles=count)
    elif scenario_type == "merge":
        data = generate_highway_merge_scenario(mainline_count=count // 2, ramp_count=count // 2)
    else:
        print(f"Unknown scenario type: {scenario_type}", file=sys.stderr)
        sys.exit(1)
        
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        
    print(f"Synthesized {scenario_type} scenario ({len(data['vehicles'])} vehicles) -> {output_path}")

if __name__ == "__main__":
    main()
