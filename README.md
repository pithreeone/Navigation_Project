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

# Setting Before Started
1. check tf between **base_frame** & **laser_frame**
2. Set the USB-port name in ___**localization_run/launch/localization.launch**___

# How to Use
- the main program of robot is achieved by finite state machine. To change state, we need events which can be created by publishing topic: /action  
- The message is user-defined in "robot_interface" package.
- Initial state is STOP

### SLAM-mode
1. Publish an topic which mission is __***start_mapping***__ (state: __***STOP***__$\rightarrow$__***START_MAPPING***__$\rightarrow$__***AUTO_MAPPING***__)
Let the topic name be **action**, or you can remap.
2. Now robot will move automatically. 
3. You can also use telop_twist_keyboard to publish velocity topic. Topic name is **control_cmd_vel**. We can use the following command to change the topic name that teleop_twist_keyboard publish.
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=control_cmd_vel`
4. Once finish mapping, publich an topic which mission is __***check_map***__. Then it will save map. The map will be saved in `/home/user-name/.ros` 
Set the following message for your topic :  
action.map_ok(bool) = true
action.set_map_name(string) = floor_XX