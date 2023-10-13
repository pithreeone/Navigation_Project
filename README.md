# Navigation_Project
This is the whole robot program which including navigation, localization and main function.

# How to install
1. git clone
編譯YDLidar-SDK  
2.  
# How to Use
- the main program of robot is achieved by finite state machine, which have multiple states. To change state, we need events which can be created by publishing topic: /action  
- The message is user-defined in "robot_interface" package.
- There are several missions:["start_mapping", "finish_control_mapping", "check_map", "choose_map", "move_goal", "move_goal_key", "record_position"]. Each will trigger corresponding events.

