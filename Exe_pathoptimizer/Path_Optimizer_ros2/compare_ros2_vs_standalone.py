#!/usr/bin/env python3
"""
Compare ROS2 output vs Standalone output for both NO_OBJECT and WITH_OBJECT scenarios
"""

import csv
import math
import sys
from typing import List, Dict, Tuple


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Convert quaternion to yaw angle in radians"""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def load_ros2_output(filename: str) -> List[Dict]:
    """Load ROS2 output file (YAML format)"""
    points = []
    
    with open(filename, 'r') as f:
        content = f.read()
    
    # Parse YAML-like structure manually
    # Split by '- time_from_start:' to get each point
    point_blocks = content.split('- time_from_start:')[1:]  # Skip header
    
    for block in point_blocks:
        try:
            point = {}
            
            # Extract position
            if 'x:' in block:
                x_match = block.split('x:')[1].split('\n')[0].strip()
                point['x'] = float(x_match)
            
            if 'y:' in block:
                # Get y value after position:
                pos_section = block.split('position:')[1] if 'position:' in block else block
                y_match = pos_section.split('y:')[1].split('\n')[0].strip()
                point['y'] = float(y_match)
            
            if 'z:' in block:
                pos_section = block.split('position:')[1] if 'position:' in block else block
                z_match = pos_section.split('z:')[1].split('\n')[0].strip()
                point['z'] = float(z_match)
            
            # Extract orientation (find the orientation section)
            if 'orientation:' in block:
                orient_section = block.split('orientation:')[1].split('longitudinal_velocity_mps:')[0]
                
                # Extract quaternion values
                if 'x:' in orient_section:
                    qx_match = orient_section.split('x:')[1].split('\n')[0].strip()
                    point['qx'] = float(qx_match)
                
                if 'y:' in orient_section:
                    qy_match = orient_section.split('y:')[1].split('\n')[0].strip()
                    point['qy'] = float(qy_match)
                
                if 'z:' in orient_section:
                    qz_match = orient_section.split('z:')[1].split('\n')[0].strip()
                    point['qz'] = float(qz_match)
                
                if 'w:' in orient_section:
                    qw_match = orient_section.split('w:')[1].split('\n')[0].strip()
                    point['qw'] = float(qw_match)
            
            # Extract velocities and other fields
            if 'longitudinal_velocity_mps:' in block:
                v_match = block.split('longitudinal_velocity_mps:')[1].split('\n')[0].strip()
                point['v'] = float(v_match)
            
            if 'lateral_velocity_mps:' in block:
                v_lat_match = block.split('lateral_velocity_mps:')[1].split('\n')[0].strip()
                point['v_lat'] = float(v_lat_match)
            
            if 'acceleration_mps2:' in block:
                accel_match = block.split('acceleration_mps2:')[1].split('\n')[0].strip()
                point['accel'] = float(accel_match)
            
            if 'heading_rate_rps:' in block:
                hr_match = block.split('heading_rate_rps:')[1].split('\n')[0].strip()
                point['heading_rate'] = float(hr_match)
            
            if 'front_wheel_angle_rad:' in block:
                fw_match = block.split('front_wheel_angle_rad:')[1].split('\n')[0].strip()
                point['front_wheel'] = float(fw_match)
            
            if 'rear_wheel_angle_rad:' in block:
                rw_match = block.split('rear_wheel_angle_rad:')[1].split('\n')[0].strip()
                point['rear_wheel'] = float(rw_match)
            
            # Calculate yaw from quaternion
            if all(k in point for k in ['qx', 'qy', 'qz', 'qw']):
                point['yaw'] = quaternion_to_yaw(point['qx'], point['qy'], point['qz'], point['qw'])
                points.append(point)
            
        except (ValueError, IndexError, AttributeError) as e:
            # Skip malformed points
            continue
    
    return points


def load_standalone_output(filename: str) -> List[Dict]:
    """Load Standalone output file (CSV)"""
    points = []
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                point = {
                    'x': float(row['x']),
                    'y': float(row['y']),
                    'z': float(row['z']),
                    'qx': float(row['qx']),
                    'qy': float(row['qy']),
                    'qz': float(row['qz']),
                    'qw': float(row['qw']),
                    'v': float(row['longitudinal_velocity_mps']),
                    'v_lat': float(row['lateral_velocity_mps']),
                    'accel': float(row['acceleration_mps2']),
                    'heading_rate': float(row['heading_rate_rps']),
                    'front_wheel': float(row['front_wheel_angle_rad']),
                    'rear_wheel': float(row['rear_wheel_angle_rad'])
                }
                point['yaw'] = quaternion_to_yaw(point['qx'], point['qy'], point['qz'], point['qw'])
                points.append(point)
            except (ValueError, KeyError):
                continue
    
    return points


def calculate_position_distance(p1: Dict, p2: Dict) -> float:
    """Calculate 3D distance between two points"""
    dx = p1['x'] - p2['x']
    dy = p1['y'] - p2['y']
    dz = p1['z'] - p2['z']
    return math.sqrt(dx*dx + dy*dy + dz*dz)


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def compare_trajectories(ros2_points: List[Dict], standalone_points: List[Dict]) -> Dict:
    """Compare two trajectories and return statistics"""
    
    min_len = min(len(ros2_points), len(standalone_points))
    
    position_diffs = []
    yaw_diffs = []
    velocity_diffs = []
    
    for i in range(min_len):
        ros2 = ros2_points[i]
        standalone = standalone_points[i]
        
        # Position difference
        pos_diff = calculate_position_distance(ros2, standalone)
        position_diffs.append(pos_diff)
        
        # Yaw difference (normalized)
        yaw_diff = abs(normalize_angle(ros2['yaw'] - standalone['yaw']))
        yaw_diffs.append(yaw_diff)
        
        # Velocity difference
        vel_diff = abs(ros2['v'] - standalone['v'])
        velocity_diffs.append(vel_diff)
    
    def calc_stats(data):
        if not data:
            return {'mean': 0, 'max': 0, 'min': 0, 'median': 0, 'std': 0}
        
        mean = sum(data) / len(data)
        
        # Calculate standard deviation
        variance = sum((x - mean) ** 2 for x in data) / len(data)
        std = math.sqrt(variance)
        
        return {
            'mean': mean,
            'max': max(data),
            'min': min(data),
            'median': sorted(data)[len(data)//2],
            'std': std
        }
    
    return {
        'ros2_points': len(ros2_points),
        'standalone_points': len(standalone_points),
        'compared_points': min_len,
        'position': calc_stats(position_diffs),
        'yaw': calc_stats(yaw_diffs),
        'velocity': calc_stats(velocity_diffs)
    }


def print_comparison_report(scenario: str, ros2_file: str, standalone_file: str, stats: Dict):
    """Print detailed comparison report"""
    
    print("\n" + "="*80)
    print(f"  {scenario}")
    print("="*80)
    print(f"\nFiles:")
    print(f"  ROS2:       {ros2_file}")
    print(f"  Standalone: {standalone_file}")
    
    print(f"\nPoint Counts:")
    print(f"  ROS2:       {stats['ros2_points']} points")
    print(f"  Standalone: {stats['standalone_points']} points")
    print(f"  Compared:   {stats['compared_points']} points")
    
    if stats['compared_points'] == 0:
        print("\n⚠️  No points to compare!")
        return
    
    print(f"\n{'Metric':<20} {'Mean':<15} {'Std Dev':<15} {'Max':<15} {'Min':<15} {'Median':<15}")
    print("-" * 95)
    
    # Position (in cm)
    pos = stats['position']
    print(f"{'Position (cm)':<20} {pos['mean']*100:>14.2f} {pos['std']*100:>14.2f} "
          f"{pos['max']*100:>14.2f} {pos['min']*100:>14.2f} {pos['median']*100:>14.2f}")
    
    # Yaw (in degrees)
    yaw = stats['yaw']
    print(f"{'Yaw (deg)':<20} {math.degrees(yaw['mean']):>14.2f} "
          f"{math.degrees(yaw['std']):>14.2f} {math.degrees(yaw['max']):>14.2f} "
          f"{math.degrees(yaw['min']):>14.2f} {math.degrees(yaw['median']):>14.2f}")
    
    # Velocity (in m/s)
    vel = stats['velocity']
    print(f"{'Velocity (m/s)':<20} {vel['mean']:>14.4f} {vel['std']:>14.4f} "
          f"{vel['max']:>14.4f} {vel['min']:>14.4f} {vel['median']:>14.4f}")
    
    # Verdict
    print("\n" + "-" * 95)
    print("Verdict:")
    
    avg_pos_diff_cm = pos['mean'] * 100
    std_pos_diff_cm = pos['std'] * 100
    max_pos_diff_cm = pos['max'] * 100
    
    avg_yaw_diff_deg = math.degrees(yaw['mean'])
    std_yaw_diff_deg = math.degrees(yaw['std'])
    max_yaw_diff_deg = math.degrees(yaw['max'])
    
    max_vel_diff = vel['max']
    
    if max_pos_diff_cm < 1.0 and max_yaw_diff_deg < 0.5 and max_vel_diff < 0.01:
        verdict = "✅ IDENTICAL - Implementations match perfectly"
    elif max_pos_diff_cm < 10.0 and max_yaw_diff_deg < 2.0 and max_vel_diff < 0.1:
        verdict = "✅ SIMILAR - Minor differences within tolerance"
    else:
        verdict = "⚠️  DIFFERENT - Significant differences detected"
    
    print(f"  {verdict}")
    print(f"  Average position error: {avg_pos_diff_cm:.2f} ± {std_pos_diff_cm:.2f} cm")
    print(f"  Max position error: {max_pos_diff_cm:.2f} cm")
    print(f"  Average yaw error: {avg_yaw_diff_deg:.2f} ± {std_yaw_diff_deg:.2f} deg")
    print(f"  Max yaw error: {max_yaw_diff_deg:.2f} deg")
    print(f"  Max velocity error: {max_vel_diff:.4f} m/s")


def main():
    print("\n" + "╔" + "="*78 + "╗")
    print("║" + " "*20 + "ROS2 vs Standalone Comparison" + " "*29 + "║")
    print("╚" + "="*78 + "╝")
    
    # Scenario 1: NO OBJECT
    print("\n" + "━"*80)
    print("  SCENARIO 1: NO OBJECT (Wide Lane)")
    print("━"*80)
    
    ros2_no_obj = "test_files_in_ros2/output_opt_path_no_object.txt"
    standalone_no_obj = "test_files_in_standalone/out_no_object_standalone.csv"
    
    try:
        ros2_points = load_ros2_output(ros2_no_obj)
        standalone_points = load_standalone_output(standalone_no_obj)
        
        stats_no_obj = compare_trajectories(ros2_points, standalone_points)
        print_comparison_report("NO OBJECT Comparison", ros2_no_obj, standalone_no_obj, stats_no_obj)
    except FileNotFoundError as e:
        print(f"\n⚠️  File not found: {e}")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    
    # Scenario 2: WITH OBJECT
    print("\n\n" + "━"*80)
    print("  SCENARIO 2: WITH OBJECT (Narrow Lane with Obstacle)")
    print("━"*80)
    
    # Note: ROS2 file has typo in name: "outpit" instead of "output"
    ros2_with_obj = "test_files_in_ros2/output_opt_path_with_object.txt"
    standalone_with_obj = "test_files_in_standalone/out_with_object_standalone.csv"
    
    try:
        ros2_points = load_ros2_output(ros2_with_obj)
        standalone_points = load_standalone_output(standalone_with_obj)
        
        stats_with_obj = compare_trajectories(ros2_points, standalone_points)
        print_comparison_report("WITH OBJECT Comparison", ros2_with_obj, standalone_with_obj, stats_with_obj)
    except FileNotFoundError as e:
        print(f"\n⚠️  File not found: {e}")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    
    # Scenario 3: STD (Real Autoware data)
    print("\n\n" + "━"*80)
    print("  SCENARIO 3: STD (Real Autoware elastic_band_smoother data)")
    print("━"*80)
    
    ros2_std = "test_files_in_ros2/output_opt_path_std.txt"
    standalone_std = "test_files_in_standalone/out_std_standalone.csv"
    
    try:
        ros2_points = load_ros2_output(ros2_std)
        standalone_points = load_standalone_output(standalone_std)
        
        stats_std = compare_trajectories(ros2_points, standalone_points)
        print_comparison_report("STD Comparison", ros2_std, standalone_std, stats_std)
    except FileNotFoundError as e:
        print(f"\n⚠️  File not found: {e}")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    
    # Summary
    print("\n\n" + "╔" + "="*78 + "╗")
    print("║" + " "*30 + "Summary" + " "*41 + "║")
    print("╚" + "="*78 + "╝")
    print("\n이 비교는 ROS2와 Standalone 구현의 일관성을 검증합니다.")
    print("두 구현 모두 동일한 OSQP 기반 MPT 최적화 알고리즘을 사용합니다.")
    print("\n시나리오별 결과:")
    print("  1. NO_OBJECT: 합성 데이터, 넓은 차선")
    print("  2. WITH_OBJECT: 합성 데이터, 좁은 차선 + 장애물")
    print("  3. STD: 실제 Autoware 데이터 (elastic_band_smoother)")
    print("\n차이가 있다면:")
    print("  • Spline interpolation 차이")
    print("  • Numerical precision 차이")
    print("  • Resampling 방법 차이")
    print("\n큰 차이가 없다면 구현이 올바르게 포팅되었음을 의미합니다.")
    print("="*80 + "\n")


if __name__ == "__main__":
    main()
