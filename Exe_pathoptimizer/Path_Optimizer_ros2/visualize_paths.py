#!/usr/bin/env python3
"""
Visualize ROS2 vs Standalone paths to understand differences
"""

import csv
import math
import matplotlib.pyplot as plt
from pathlib import Path

def quaternion_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

def parse_ros2_output(filepath):
    points = []
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    i = 0
    while i < len(lines):
        if 'position:' in lines[i]:
            x = float(lines[i+1].split(':')[1].strip())
            y = float(lines[i+2].split(':')[1].strip())
            qx = float(lines[i+5].split(':')[1].strip())
            qy = float(lines[i+6].split(':')[1].strip())
            qz = float(lines[i+7].split(':')[1].strip())
            qw = float(lines[i+8].split(':')[1].strip())
            yaw = quaternion_to_yaw(qx, qy, qz, qw)
            points.append({'x': x, 'y': y, 'yaw': yaw})
            i += 10
        else:
            i += 1
    return points

def parse_standalone_output(filepath):
    points = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            points.append({
                'x': float(row['x']),
                'y': float(row['y']),
                'yaw': float(row['yaw'])
            })
    return points

def parse_reference_path(filepath):
    points = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            points.append({
                'x': float(row['x']),
                'y': float(row['y'])
            })
    return points

# Parse files
ros2_file = Path(__file__).parent.parent / "autoware_path_optimizer" / "test_output.txt"
standalone_file = Path(__file__).parent / "optimized_trajectory.csv"
reference_file = Path(__file__).parent / "test_path_from_ros.csv"

ros2_points = parse_ros2_output(ros2_file)
standalone_points = parse_standalone_output(standalone_file)
reference_points = parse_reference_path(reference_file)

# Create visualization
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

# Plot 1: Full paths
ax1.plot([p['x'] for p in ros2_points], [p['y'] for p in ros2_points], 
         'b-', label='ROS2', linewidth=2, alpha=0.7)
ax1.plot([p['x'] for p in standalone_points], [p['y'] for p in standalone_points], 
         'r--', label='Standalone', linewidth=2, alpha=0.7)
ax1.plot([p['x'] for p in reference_points], [p['y'] for p in reference_points], 
         'go--', label='Reference Path', markersize=10, linewidth=1)
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_title('Full Path Comparison')
ax1.legend()
ax1.grid(True)
ax1.axis('equal')

# Plot 2: First 20m (zoomed in)
ax2.plot([p['x'] for p in ros2_points[:40]], [p['y'] for p in ros2_points[:40]], 
         'b-', label='ROS2', linewidth=2, alpha=0.7)
ax2.plot([p['x'] for p in standalone_points[:40]], [p['y'] for p in standalone_points[:40]], 
         'r--', label='Standalone', linewidth=2, alpha=0.7)
ax2.plot([p['x'] for p in reference_points], [p['y'] for p in reference_points], 
         'go--', label='Reference Path', markersize=10, linewidth=1)
ax2.set_xlabel('X (m)')
ax2.set_ylabel('Y (m)')
ax2.set_title('First 20m (Zoomed)')
ax2.legend()
ax2.grid(True)
ax2.axis('equal')

plt.tight_layout()
plt.savefig('path_comparison.png', dpi=150)
print(f"Saved visualization to path_comparison.png")

# Print statistics
print(f"\nPath Statistics:")
print(f"Reference points: {len(reference_points)}")
print(f"ROS2 points: {len(ros2_points)}")
print(f"Standalone points: {len(standalone_points)}")
print(f"\nFirst 3 points comparison:")
for i in range(min(3, len(ros2_points), len(standalone_points))):
    print(f"Point {i}:")
    print(f"  ROS2:       x={ros2_points[i]['x']:7.4f}, y={ros2_points[i]['y']:7.4f}")
    print(f"  Standalone: x={standalone_points[i]['x']:7.4f}, y={standalone_points[i]['y']:7.4f}")
    print(f"  Difference: Δx={abs(ros2_points[i]['x'] - standalone_points[i]['x']):7.4f}, Δy={abs(ros2_points[i]['y'] - standalone_points[i]['y']):7.4f}")
