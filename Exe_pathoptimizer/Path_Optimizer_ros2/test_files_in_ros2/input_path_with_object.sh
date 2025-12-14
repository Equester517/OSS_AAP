#!/bin/bash
# Scenario 2: Path WITH Object (Narrowed drivable area)
#
# This scenario simulates an obstacle in the left side of the lane.
# Object location: x = -0.8, y = 25.0 ~ 35.0 (10m long obstacle)
# 
# Drivable Area Changes:
#   y = 0~20m:   Normal width (4.0m: -2.0 to +2.0)
#   y = 20~25m:  Transition zone (narrowing from -2.0 to -0.3)
#   y = 25~35m:  Narrowed (2.3m: -0.3 to +2.0) ← Object avoidance zone
#   y = 35~40m:  Transition zone (widening from -0.3 to -2.0)
#   y = 40~60m:  Normal width (4.0m: -2.0 to +2.0)
#
# Reference Path:
#   Same as no-object scenario, but path optimizer should adjust
#   to stay within the narrowed bounds around y=25~35m
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
    {x: -1.2, y: 22.5, z: 0.0},
    {x: -0.3, y: 25.0, z: 0.0},
    {x: -0.3, y: 27.5, z: 0.0},
    {x: -0.3, y: 30.0, z: 0.0},
    {x: -0.3, y: 32.5, z: 0.0},
    {x: -0.3, y: 35.0, z: 0.0},
    {x: -1.2, y: 37.5, z: 0.0},
    {x: -2.0, y: 40.0, z: 0.0},
    {x: -2.0, y: 50.0, z: 0.0},
    {x: -2.0, y: 60.0, z: 0.0}
  ],
  right_bound: [
    {x: 2.0, y: 0.0, z: 0.0},
    {x: 2.0, y: 10.0, z: 0.0},
    {x: 2.0, y: 20.0, z: 0.0},
    {x: 2.0, y: 22.5, z: 0.0},
    {x: 2.0, y: 25.0, z: 0.0},
    {x: 2.0, y: 27.5, z: 0.0},
    {x: 2.0, y: 30.0, z: 0.0},
    {x: 2.0, y: 32.5, z: 0.0},
    {x: 2.0, y: 35.0, z: 0.0},
    {x: 2.0, y: 37.5, z: 0.0},
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
