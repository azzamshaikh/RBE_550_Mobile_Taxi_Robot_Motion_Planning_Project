# RBE 550: Waste Taxi Mobile Robot Motion Planning Project

This repository contains the code developed for the RBE 550 group project. The project focuses on the development of motion planning algorithms for a mobile robot to taxi waste in a warehouse.

A full write up of the project is available in the `docs` folder.

## Team Members

Azzam Shaikh, Sidhant Karamchandani, and Tamir Lieber 

## Dependencies

ROS Noetic and Ubuntu 20.04 is being used for this.

Install the following Turtlebot3 packages and dependencies:

```
sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3
sudo apt-get install ros-noetic-navigation
```

## Build instructions

### Pull repo
```
cd ~
git clone https://github.com/azzamshaikh/RBE_550_Mobile_Taxi_Robot_Motion_Planning_Project.git catkin_ws
cd catkin_ws
catkin_make
```
### Source workspace

```
source devel/setup.bash
```
### Declare the Turtlebot3 burger model. 

```
export TURTLEBOT3_MODEL=waffle_pi
```

This needs to be done for every terminal that is created. This can be added to the bottom of the ~/.bashrc file so you dont need to add it everytime. 

###  Open Gazebo environment

```
roslaunch turtlebot3_gazebo warehouse.launch
```

### Open Navigation/RVIZ

In a new terminal, source your workspace (and add the waffle_pi model command if it is not appended to your bashrc file). 

To run the Dijkstra planner, run the following command:

```
roslaunch turtlebot3_gazebo dijkstra_navigation.launch
```

To run the A* planner, run the following command:

```
roslaunch turtlebot3_gazebo astar_navigation.launch
```

To run the RRT planner, run the following command:

```
roslaunch turtlebot3_gazebo rrt_navigation.launch
```

To run the PRM planner, run the following command:

```
roslaunch turtlebot3_gazebo PRM_navigation.launch
```

To run the default planner, run the following command:

```
roslaunch turtlebot3_gazebo turtlebot3_warehouse_nav.launch
```

In RVIZ, you can send a 2D Nav Goal command to make the robot move. 

### Run Taxi Service

In a new terminal, source your space (and add the waffle_pi model command if it is not appended to your bashrc file). Then run the following command

```
roslaunch taxi_service taxi_service.launch
```

## Demonstrations

### Dijkstra Planner

![](./docs/media/dijkstra_demo.gif)

### A* Planner

![](./docs/media/astar_demo.gif)

### RRT Planner

![](./docs/media/rrt_demo.gif)

### PRM Planner

![](./docs/media/prm_demo.gif)
