#!/usr/bin/env python3
"""
Convert ROS2 test data (YAML format) to CSV format for standalone Path_Optimizer

Usage:
  python3 convert_ros_to_csv.py test_input_path.txt test_path_from_ros.csv
"""

import sys
import yaml
import math

def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion to yaw angle"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

def convert_path_yaml_to_csv(yaml_file, csv_file):
    """Convert ROS2 path YAML to CSV format
    
    Input can be:
    1. ros2 topic pub command format
    2. Pure YAML format
    
    CSV output format:
      x,y,yaw,velocity
    """
    print(f"Reading {yaml_file}...")
    
    with open(yaml_file, 'r') as f:
        content = f.read()
    
    # Check if it's a ros2 topic pub command
    if content.strip().startswith('ros2 topic pub'):
        # Extract YAML content from command
        import re
        
        # Find the YAML part (between first { and last })
        match = re.search(r'\{(.+)\}', content, re.DOTALL)
        if not match:
            print("ERROR: Could not extract YAML from ros2 command")
            return False
        
        yaml_content = '{' + match.group(1) + '}'
        
        # Convert simplified YAML format to proper format
        # Replace single-quoted keys with double-quoted
        yaml_content = yaml_content.replace("'", '"')
        
        # Extract points array
        points_match = re.search(r'points:\s*\[(.*?)\](?:\s*}|$)', yaml_content, re.DOTALL)
        if not points_match:
            print("ERROR: Could not find points array")
            return False
        
        points_str = points_match.group(1)
        
        # Parse each point manually
        points = []
        # Split by '},' to get individual points
        point_strs = points_str.split('},')
        
        for point_str in point_strs:
            # Extract position
            x_match = re.search(r'x:\s*(-?[\d.]+)', point_str)
            y_match = re.search(r'y:\s*(-?[\d.]+)', point_str)
            
            # Extract orientation (simplified: assume w: 1.0 means yaw=0)
            w_match = re.search(r'orientation:\s*\{[^}]*w:\s*(-?[\d.]+)', point_str)
            x_ori_match = re.search(r'orientation:\s*\{[^}]*x:\s*(-?[\d.]+)', point_str)
            y_ori_match = re.search(r'orientation:\s*\{[^}]*y:\s*(-?[\d.]+)', point_str)
            z_ori_match = re.search(r'orientation:\s*\{[^}]*z:\s*(-?[\d.]+)', point_str)
            
            # Extract velocity
            vel_match = re.search(r'longitudinal_velocity_mps:\s*(-?[\d.]+)', point_str)
            
            if x_match and y_match:
                x = float(x_match.group(1))
                y = float(y_match.group(1))
                
                # Calculate yaw from quaternion
                if w_match:
                    w = float(w_match.group(1))
                    ori_x = float(x_ori_match.group(1)) if x_ori_match else 0.0
                    ori_y = float(y_ori_match.group(1)) if y_ori_match else 0.0
                    ori_z = float(z_ori_match.group(1)) if z_ori_match else 0.0
                    
                    yaw = quaternion_to_yaw(ori_x, ori_y, ori_z, w)
                else:
                    yaw = 0.0
                
                velocity = float(vel_match.group(1)) if vel_match else 10.0
                
                points.append({
                    'x': x,
                    'y': y,
                    'yaw': yaw,
                    'velocity': velocity
                })
        
        print(f"Found {len(points)} poses")
        
        # Also extract bounds
        left_bound_match = re.search(r'left_bound:\s*\[(.*?)\]', yaml_content, re.DOTALL)
        right_bound_match = re.search(r'right_bound:\s*\[(.*?)\]', yaml_content, re.DOTALL)
        
        if left_bound_match and right_bound_match:
            # Parse bounds
            left_bound_str = left_bound_match.group(1)
            right_bound_str = right_bound_match.group(1)
            
            left_points = []
            right_points = []
            
            for bound_str in left_bound_str.split('},'):
                x_match = re.search(r'x:\s*(-?[\d.]+)', bound_str)
                y_match = re.search(r'y:\s*(-?[\d.]+)', bound_str)
                if x_match and y_match:
                    left_points.append({
                        'x': float(x_match.group(1)),
                        'y': float(y_match.group(1))
                    })
            
            for bound_str in right_bound_str.split('},'):
                x_match = re.search(r'x:\s*(-?[\d.]+)', bound_str)
                y_match = re.search(r'y:\s*(-?[\d.]+)', bound_str)
                if x_match and y_match:
                    right_points.append({
                        'x': float(x_match.group(1)),
                        'y': float(y_match.group(1))
                    })
            
            # Save bounds
            if left_points:
                with open('test_left_bound.csv', 'w') as f:
                    f.write("x,y\n")
                    for p in left_points:
                        f.write(f"{p['x']},{p['y']}\n")
                print(f"Saved {len(left_points)} left bound points to test_left_bound.csv")
            
            if right_points:
                with open('test_right_bound.csv', 'w') as f:
                    f.write("x,y\n")
                    for p in right_points:
                        f.write(f"{p['x']},{p['y']}\n")
                print(f"Saved {len(right_points)} right bound points to test_right_bound.csv")
    else:
        # Try parsing as pure YAML
        data = yaml.safe_load(content)
        
        if not isinstance(data, list):
            print(f"ERROR: Expected list of poses, got {type(data)}")
            return False
        
        print(f"Found {len(data)} poses")
        
        points = []
        for item in data:
            if 'pose' in item:
                pose = item['pose']
            elif 'position' in item and 'orientation' in item:
                pose = item
            else:
                print(f"WARNING: Unknown format: {item.keys()}")
                continue
            
            pos = pose['position']
            ori = pose['orientation']
            
            x = pos['x']
            y = pos['y']
            yaw = quaternion_to_yaw(ori.get('x', 0.0), ori.get('y', 0.0), 
                                   ori.get('z', 0.0), ori.get('w', 1.0))
            
            # Default velocity
            velocity = item.get('longitudinal_velocity_mps', 10.0)
            
            points.append({
                'x': x,
                'y': y,
                'yaw': yaw,
                'velocity': velocity
            })
    
    # Write CSV
    with open(csv_file, 'w') as f:
        f.write("x,y,yaw,velocity\n")
        
        for p in points:
            f.write(f"{p['x']},{p['y']},{p['yaw']},{p['velocity']}\n")
    
    print(f"Wrote {len(points)} points to {csv_file}")
    return True

def convert_odometry_yaml_to_params(yaml_file):
    """Extract ego pose from ROS2 odometry YAML
    
    YAML format (nav_msgs/Odometry):
      header:
        stamp: ...
      pose:
        pose:
          position: {x: ..., y: ..., z: ...}
          orientation: {x: ..., y: ..., z: ..., w: ...}
      twist:
        twist:
          linear: {x: ..., y: ..., z: ...}
    """
    print(f"\nReading {yaml_file}...")
    
    with open(yaml_file, 'r') as f:
        content = f.read()
    
    data = yaml.safe_load(content)
    
    if 'pose' in data and 'pose' in data['pose']:
        pose = data['pose']['pose']
        pos = pose['position']
        ori = pose['orientation']
        
        yaw = quaternion_to_yaw(ori['x'], ori['y'], ori['z'], ori['w'])
        
        print(f"Ego pose:")
        print(f"  position: ({pos['x']:.3f}, {pos['y']:.3f}, {pos['z']:.3f})")
        print(f"  orientation: ({ori['x']:.3f}, {ori['y']:.3f}, {ori['z']:.3f}, {ori['w']:.3f})")
        print(f"  yaw: {yaw:.3f} rad ({math.degrees(yaw):.1f} deg)")
        
        if 'twist' in data and 'twist' in data['twist']:
            twist = data['twist']['twist']
            linear = twist['linear']
            velocity = math.sqrt(linear['x']**2 + linear['y']**2)
            print(f"  velocity: {velocity:.3f} m/s")
        
        return {
            'x': pos['x'],
            'y': pos['y'],
            'z': pos['z'],
            'qx': ori['x'],
            'qy': ori['y'],
            'qz': ori['z'],
            'qw': ori['w'],
            'yaw': yaw
        }
    
    print("ERROR: Could not parse odometry data")
    return None

def main():
    if len(sys.argv) < 3:
        print("Usage:")
        print("  Convert path:")
        print("    python3 convert_ros_to_csv.py test_input_path.txt test_path_from_ros.csv")
        print("  Extract ego pose:")
        print("    python3 convert_ros_to_csv.py test_input_odometry.txt")
        sys.exit(1)
    
    input_file = sys.argv[1]
    
    if 'odometry' in input_file.lower():
        # Extract ego pose parameters
        ego_params = convert_odometry_yaml_to_params(input_file)
        if ego_params:
            print("\nEgo parameters extracted successfully")
    else:
        # Convert path to CSV
        if len(sys.argv) < 3:
            print("ERROR: Output CSV file required")
            sys.exit(1)
        
        output_file = sys.argv[2]
        success = convert_path_yaml_to_csv(input_file, output_file)
        
        if success:
            print("\nConversion successful!")
        else:
            print("\nConversion failed!")
            sys.exit(1)

if __name__ == "__main__":
    main()
