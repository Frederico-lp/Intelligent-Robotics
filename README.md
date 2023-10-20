## Prerequisites
- [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)

## How to compile:
In the `catkin_ws/` directory:
```bash
catkin_make
```

## How to run
Everytime you open a new terminal, source the setup files (or add the commands to your .bashrc):
```bash
source /opt/ros/noetic/setup.bash
```
```bash
source ./devel/setup.bash
```
It can be convenient to automatically source this script every time a new shell is launched. These commands will do that for you.
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ./devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Start the ROS core:
```bash
roscore &
```

In a new terminal, launch the simulation:
```bash
bash ./run_sim.bash
```

In another new termina, start the robot movement:
```bash
python3 ./src/robot_gazebo/scripts/move.py
```