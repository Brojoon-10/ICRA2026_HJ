#!/bin/bash
# Reset ego to start line (s=0). gazebo_wall_2 global_waypoints.json wp[0].
# Coords confirmed in earlier session via the user.
rosservice call /gazebo/set_model_state "{model_state: {model_name: 'unicorn',
  pose: {position: {x: -7.117928, y: -0.630808, z: 0.5},
         orientation: {x: 0.0, y: 0.0, z: -0.757395, w: 0.652957}},
  twist: {linear: {x: 0.0, y: 0.0, z: 0.0},
          angular: {x: 0.0, y: 0.0, z: 0.0}},
  reference_frame: 'world'}}" >/dev/null
echo "ego respawned at s=0"
