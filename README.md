# RBE_550_Mobile_Taxi_Robot_Motion_Planning_Project

## Build instructions

### Pull repo
```
cd ~
git clone https://github.com/azzamshaikh/RBE_550_Mobile_Taxi_Robot_Motion_Planning_Project.git catkin_ws
cd catkin_ws
catkin_make
```
### Source workspace

`source devel/setup.bash`

### Declare the Turtlebot3 burger model. 

`export TURTLEBOT3_MODEL=burger`

This needs to be done for every terminal that is created. This can be added to the bottom of the ~/.bashrc file so you dont need to add it everytime. 

###  Open Gazebo environment

`roslaunch turtlebot3_gazebo turtlebot3_warehouse.launch`

### Open Navigation/RVIZ

In a new terminal, source your workspace (and add the burger model command if it is not appended to your bashrc file). Then run the following command

`roslaunch turtlebot3_gazebo turtlebot3_warehouse_nav.launch`

In RVIZ, you can send a 2D Nav Goal command to make the robot move. 
