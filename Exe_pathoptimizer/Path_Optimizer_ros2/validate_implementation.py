#!/usr/bin/env python3
"""
Validation script to compare ROS2 autoware_path_optimizer output 
with Standalone Path_Optimizer output.
"""

import csv
import math
import sys
from pathlib import Path

def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion to yaw angle in radians."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

def parse_ros2_output(filepath):
    """Parse ROS2 test_output.txt (YAML format)."""
    points = []
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    i = 0
    while i < len(lines):
        if 'position:' in lines[i]:
            # Parse position
            x = float(lines[i+1].split(':')[1].strip())
            y = float(lines[i+2].split(':')[1].strip())
            
            # Parse orientation (next 5 lines)
            qx = float(lines[i+5].split(':')[1].strip())
            qy = float(lines[i+6].split(':')[1].strip())
            qz = float(lines[i+7].split(':')[1].strip())
            qw = float(lines[i+8].split(':')[1].strip())
            
            yaw = quaternion_to_yaw(qx, qy, qz, qw)
            
            # Parse velocity
            vel = float(lines[i+9].split(':')[1].strip())
            
            points.append({
                'x': x,
                'y': y,
                'yaw': yaw,
                'velocity': vel
            })
            i += 10
        else:
            i += 1
    
    return points

def parse_standalone_output(filepath):
    """Parse Standalone optimized_trajectory.csv."""
    points = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            points.append({
                'x': float(row['x']),
                'y': float(row['y']),
                'yaw': float(row['yaw']),
                'velocity': float(row['velocity'])
            })
    return points

def compute_errors(ros2_points, standalone_points):
    """Compute position, yaw, and velocity errors."""
    # Match closest points by y-coordinate (since path goes north)
    errors = []
    
    for sp in standalone_points:
        # Find closest ROS2 point by y-coordinate
        closest_rp = min(ros2_points, key=lambda p: abs(p['y'] - sp['y']))
        
        pos_error = math.sqrt((sp['x'] - closest_rp['x'])**2 + 
                              (sp['y'] - closest_rp['y'])**2)
        
        yaw_error = abs(sp['yaw'] - closest_rp['yaw']) * 180.0 / math.pi
        
        vel_error = abs(sp['velocity'] - closest_rp['velocity'])
        
        errors.append({
            'standalone': sp,
            'ros2': closest_rp,
            'pos_error': pos_error,
            'yaw_error': yaw_error,
            'vel_error': vel_error
        })
    
    return errors

def print_comparison(errors, num_samples=10):
    """Print comparison results."""
    print("\n" + "="*80)
    print("VALIDATION RESULTS: ROS2 vs Standalone")
    print("="*80)
    
    # Statistics
    pos_errors = [e['pos_error'] for e in errors]
    yaw_errors = [e['yaw_error'] for e in errors]
    vel_errors = [e['vel_error'] for e in errors]
    
    print(f"\nTotal points compared: {len(errors)}")
    print(f"\n{'Metric':<20} {'Mean':<12} {'Max':<12} {'Status'}")
    print("-"*60)
    
    pos_mean = sum(pos_errors) / len(pos_errors)
    pos_max = max(pos_errors)
    pos_status = "✓ PASS" if pos_max < 0.1 else "✗ FAIL"
    print(f"{'Position Error (m)':<20} {pos_mean:<12.6f} {pos_max:<12.6f} {pos_status}")
    
    yaw_mean = sum(yaw_errors) / len(yaw_errors)
    yaw_max = max(yaw_errors)
    yaw_status = "✓ PASS" if yaw_max < 5.0 else "✗ FAIL"
    print(f"{'Yaw Error (deg)':<20} {yaw_mean:<12.6f} {yaw_max:<12.6f} {yaw_status}")
    
    vel_mean = sum(vel_errors) / len(vel_errors)
    vel_max = max(vel_errors)
    vel_status = "✓ PASS" if vel_max < 0.5 else "✗ FAIL"
    print(f"{'Velocity Error (m/s)':<20} {vel_mean:<12.6f} {vel_max:<12.6f} {vel_status}")
    
    # Sample comparison
    print(f"\n{'Sample Points (every {len(errors)//num_samples}th point)':^80}")
    print("-"*80)
    print(f"{'Idx':<5} {'ROS2 (x, y, yaw)':<30} {'Standalone (x, y, yaw)':<30} {'Δpos':<10}")
    print("-"*80)
    
    step = max(1, len(errors) // num_samples)
    for i in range(0, len(errors), step):
        e = errors[i]
        ros2_str = f"({e['ros2']['x']:.3f}, {e['ros2']['y']:.3f}, {e['ros2']['yaw']*180/math.pi:.1f}°)"
        standalone_str = f"({e['standalone']['x']:.3f}, {e['standalone']['y']:.3f}, {e['standalone']['yaw']*180/math.pi:.1f}°)"
        print(f"{i:<5} {ros2_str:<30} {standalone_str:<30} {e['pos_error']:.6f}")
    
    # Overall verdict
    print("\n" + "="*80)
    if pos_status == "✓ PASS" and yaw_status == "✓ PASS" and vel_status == "✓ PASS":
        print("VERDICT: ✓ IMPLEMENTATION VALIDATED")
        print("Standalone Path_Optimizer produces identical results to ROS2 version.")
    else:
        print("VERDICT: ✗ IMPLEMENTATION DIFFERS")
        print("Standalone Path_Optimizer output deviates from ROS2 version.")
        print("\nPossible causes:")
        if pos_status == "✗ FAIL":
            print("  - Lateral offset calculation differs")
            print("  - State equation formulation issues")
        if yaw_status == "✗ FAIL":
            print("  - Orientation computation differs")
            print("  - Quaternion conversion issues")
        if vel_status == "✗ FAIL":
            print("  - Velocity propagation issues")
    print("="*80 + "\n")
    
    return pos_status == "✓ PASS" and yaw_status == "✓ PASS" and vel_status == "✓ PASS"

def main():
    ros2_file = Path(__file__).parent.parent / "autoware_path_optimizer" / "test_output.txt"
    standalone_file = Path(__file__).parent / "optimized_trajectory.csv"
    
    if not ros2_file.exists():
        print(f"Error: ROS2 output file not found: {ros2_file}")
        print("Please run ROS2 version first to generate test_output.txt")
        return 1
    
    if not standalone_file.exists():
        print(f"Error: Standalone output file not found: {standalone_file}")
        print("Please run Standalone version first:")
        print("  cd Path_Optimizer")
        print("  ./build/path_optimizer test_path_from_ros.csv test_left_bound.csv test_right_bound.csv")
        return 1
    
    print("Loading outputs...")
    ros2_points = parse_ros2_output(ros2_file)
    standalone_points = parse_standalone_output(standalone_file)
    
    print(f"ROS2 points: {len(ros2_points)}")
    print(f"Standalone points: {len(standalone_points)}")
    
    if len(ros2_points) == 0 or len(standalone_points) == 0:
        print("Error: No points found in output files")
        return 1
    
    errors = compute_errors(ros2_points, standalone_points)
    success = print_comparison(errors)
    
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())
