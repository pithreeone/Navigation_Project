# Navigation Project Meta Package

This is the whole robot program which including navigation, localization and main function.

## 1. How to install

### 1.1. clone the meta package

1. `git clone https://github.com/pithreeone/Navigation_Project.git`  
   or clone by ssh `git clone git@github.com:pithreeone/Navigation_Project.git`  

### 1.2. compile YDLidar-SDK  

1. `cd ~/."your_ws"/src/Navigation_Project/localization/YDLidar-SDK/build`
2. `cmake ..`
3. `make`
4. `sudo make install`

### 1.3. install some binary file

1. `sudo apt-get install ros-noetic-gazebo-ros`
2. `sudo apt-get install ros-noetic-ros-controllers`
3. `sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control`

## 2. Setting Before Start

1. check tf between `base_frame` & `laser_frame`
2. Set the USB-port name in `localization_run/launch/localization.launch`. Check the following argument:
    - lidar_port
    - odometry_port 

## 3. How to Use

- the main program of robot is achieved by finite state machine. To change state, we need events which can be created by publishing topic: `/action`  
- The message is user-defined in `robot_interface` package.
- Initial state is `STOP`

### 3.1. SLAM-mode

1. Publish an topic which mission is `start_mapping`
(state: `STOP`$\rightarrow$`START_MAPPING`$\rightarrow$`AUTO_MAPPING`)
Let the topic name be `action`, or you can remap.
2. Now robot will move automatically. 
3. You can also use telop_twist_keyboard to publish velocity topic. Topic name is `control_cmd_vel`. We can use the following command to change the topic name that teleop_twist_keyboard publish.
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=control_cmd_vel`
4. Once finish mapping, publich an topic which mission is `check_map`. Then it will save map. The map will be saved in `/home/user-name/.ros` 
Set the following message for your topic :  
    - `action.map_ok(bool) = true`
    - `action.set_map_name(string) = floor_XX`

## 4. The messages
The package provides three custom message types. All of their numerical values are provided in SI units.

- `Interface`
  - `string mission` missions:[`start_mapping`, `finish_control_mapping`, `check_map`, `choose_map`, `move_goal`, `move_goal_key`, `record_position`]
  - `geometry_msgs/PoseStamped goal` if mission is `move_goal`, set goal
  - `string choose_map_name` if mission is `choose_map`, choose a map
  - `string position_key` if mission is `record_position`, set name of position
  - `bool map_ok` check whether map is acceptable
  - `string set_map_name` if mission is `check_map`, set map name  
  
- `Robot_State`
  - `string robot_state` robot current state

## 5. The launch files

### 5.1 Run real robot
For start up real robot, you can launch the file below.
- `roslaunch navigation_run navigation.launch 2>/dev/null`

### 5.2 Testing tool

#### 5.3.1 Publish command to robot 

publish_mission node can help you publish the Interface topic. It's hard to publish it using `rostopic pub ...`
- `rosrun navigation_run publish_mission`

*Be careful to publish the command velocity to odometry with topic name `control_cmd_vel` instead of `cmd_vel`
- `rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=control_cmd_vel`

#### 5.3.2 Check connection with firmware

 - `roslaunch localization_run odometry.launch`  

You can add specific port name after the above command.  
 - `roslaunch localization_run odometry.launch odometry_port:=/dev/USB1`
