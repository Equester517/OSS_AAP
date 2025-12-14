#!/bin/bash
# Odometry Input for autoware_path_optimizer
#
# This provides the ego vehicle's current state:
#   - Position: (0.0, 0.0, 0.0) at the start of the path
#   - Orientation: Facing forward (yaw = 0)
#   - Velocity: 5.0 m/s (18 km/h)
#
# This input is the same for both scenarios (with/without object)
# because the ego vehicle state doesn't change based on downstream obstacles.
source /home/bskang/autoware/install/setup.bash

ros2 topic pub /localization/kinematic_state nav_msgs/msg/Odometry \
"{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'map'
  },
  child_frame_id: 'base_link',
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  },
  twist: {
    twist: {
      linear: {x: 5.0, y: 0.0, z: 0.0},
      angular: {x: 0.0, y: 0.0, z: 0.0}
    },
    covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  }
}" --rate 10
