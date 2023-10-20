## Prerequisites
- [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)

## How to compile:
In the `catkin_ws/` directory:
```bash
catkin_make
```

## How to run
First, **make sure** you are in the `catkin_ws/` directory.

Everytime you open a new terminal, source the setup files:
```bash
source /opt/ros/noetic/setup.bash
```
```bash
source ./devel/setup.bash
```
(Alternatively) It can be very convenient to automatically source every time a new terminal is opened:
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