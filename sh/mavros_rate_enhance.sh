#! /bin/bash
# improve imu rate
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0

sleep 8
# improve odom rate
rosrun mavros mavcmd long 511 32 5000 0 0 0 0 0

echo "done"

