# Navigation_Project
This is the whole robot program which including navigation, localization and main function.

# How to install
### clone the meta package
1. `git clone https://github.com/pithreeone/Navigation_Project.git`  
   or clone by ssh `git clone git@github.com:pithreeone/Navigation_Project.git`  
### compile YDLidar-SDK  
1. `cd ~/."your_ws"/src/Navigation_Project/localization/YDLidar-SDK/build`
2. `cmake ..`
3. `make`
4. `sudo make install`
### install some binary file
1. `sudo apt-get install ros-noetic-gazebo-ros`
2. `sudo apt-get install ros-noetic-ros-controllers`
3. `sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control`


# How to Use
- the main program of robot is achieved by finite state machine, which have multiple states. To change state, we need events which can be created by publishing topic: /action  
- The message is user-defined in "robot_interface" package.
- There are several missions:["start_mapping", "finish_control_mapping", "check_map", "choose_map", "move_goal", "move_goal_key", "record_position"]. Each will trigger corresponding events.
