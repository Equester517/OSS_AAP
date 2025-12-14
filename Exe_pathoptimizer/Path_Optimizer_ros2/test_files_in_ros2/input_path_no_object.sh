#!/bin/bash
# Scenario 1: Path WITHOUT Object (Wide drivable area)
# 
# This scenario simulates a clear road with no obstacles.
# The vehicle can use the full lane width (4.0m total: -2.0 to +2.0)
# 
# Drivable Area:
#   left_bound:  x = -2.0 (left lane boundary)
#   right_bound: x = +2.0 (right lane boundary)
#   width: 4.0m (full lane width)
#
# Reference Path:
#   Slightly curved path (sinusoidal) from y=0 to y=60m
#   Designed to test path smoothing and optimization
source /home/bskang/autoware/install/setup.bash

ros2 topic pub /input/path autoware_planning_msgs/msg/Path \
"{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'map'
  },
  left_bound: [
    {x: -2.0, y: 0.0, z: 0.0},
    {x: -2.0, y: 10.0, z: 0.0},
    {x: -2.0, y: 20.0, z: 0.0},
    {x: -2.0, y: 30.0, z: 0.0},
    {x: -2.0, y: 40.0, z: 0.0},
    {x: -2.0, y: 50.0, z: 0.0},
    {x: -2.0, y: 60.0, z: 0.0}
  ],
  right_bound: [
    {x: 2.0, y: 0.0, z: 0.0},
    {x: 2.0, y: 10.0, z: 0.0},
    {x: 2.0, y: 20.0, z: 0.0},
    {x: 2.0, y: 30.0, z: 0.0},
    {x: 2.0, y: 40.0, z: 0.0},
    {x: 2.0, y: 50.0, z: 0.0},
    {x: 2.0, y: 60.0, z: 0.0}
  ],
  points: [
    {
      pose: {
        position: {x: 0.0, y: 0.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      longitudinal_velocity_mps: 8.33,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: false
    },
    {
      pose: {
        position: {x: 0.3, y: 5.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      longitudinal_velocity_mps: 8.33,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: false
    },
    {
      pose: {
        position: {x: 0.5, y: 10.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      longitudinal_velocity_mps: 8.33,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: false
    },
    {
      pose: {
        position: {x: 0.3, y: 15.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      longitudinal_velocity_mps: 8.33,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: false
    },
    {
      pose: {
        position: {x: 0.0, y: 20.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      longitudinal_velocity_mps: 8.33,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: false
    },
    {
      pose: {
        position: {x: -0.3, y: 25.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      longitudinal_velocity_mps: 8.33,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: false
    },
    {
      pose: {
        position: {x: -0.5, y: 30.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      longitudinal_velocity_mps: 8.33,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: false
    },
    {
      pose: {
        position: {x: -0.3, y: 35.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      longitudinal_velocity_mps: 8.33,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: false
    },
    {
      pose: {
        position: {x: 0.0, y: 40.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      longitudinal_velocity_mps: 8.33,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: false
    },
    {
      pose: {
        position: {x: 0.3, y: 45.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      longitudinal_velocity_mps: 8.33,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: false
    },
    {
      pose: {
        position: {x: 0.5, y: 50.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      longitudinal_velocity_mps: 8.33,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: false
    },
    {
      pose: {
        position: {x: 0.3, y: 55.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      longitudinal_velocity_mps: 8.33,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: false
    },
    {
      pose: {
        position: {x: 0.0, y: 60.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      longitudinal_velocity_mps: 8.33,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    }
  ]
}" --once
