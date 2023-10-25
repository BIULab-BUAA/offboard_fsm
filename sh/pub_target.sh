#!/bin/bash
rostopic pub -r 4 /position_cmd quadrotor_msgs/PositionCommand "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
position: {x: 0.0, y: 0.0, z: 2.0}
velocity: {x: 0.0, y: 0.0, z: 0.0}
acceleration: {x: 0.0, y: 0.0, z: 0.0}
jerk: {x: 0.0, y: 0.0, z: 0.0}
yaw: 0.0
yaw_dot: 0.0
kx: [0.0, 0.0, 0.0]
kv: [0.0, 0.0, 0.0]
trajectory_id: 0
trajectory_flag: 0" 

