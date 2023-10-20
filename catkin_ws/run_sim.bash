#! /bin/bash

# Generate x
random_integer_x=$((RANDOM % 5801 + 11200))
random_float_x=$(bc -l <<< "scale=2; ($random_integer_x / 10000.0) - 2.8")
echo "Random Value: $random_float_x"

# Generate y
random_integer_y=$((RANDOM % 4001))
random_float_y=$(bc -l <<< "scale=2; ($random_integer_y / 10000) - 0.4")
echo "Random Value: $random_float_y"

# Generate yaw
random_value=$RANDOM
max_value=6.28318  # 2π
random_float_yaw=$(bc -l <<< "$random_value / 32767 * $max_value")
echo "Random Value: $random_float_yaw"

echo "roslaunch robot_gazebo robot.launch x:=$random_float_x y:=$random_float_y z:=0 yaw:=$random_float_yaw"
roslaunch robot_gazebo robot.launch x:=$random_float_x y:=$random_float_y z:=0 yaw:=$random_float_yaw
