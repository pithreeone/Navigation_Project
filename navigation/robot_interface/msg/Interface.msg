# missions:["start_mapping", "finish_control_mapping", "check_map", "move_goal", "record_position"]
string mission

# if mission is "move_goal", set goal
geometry_msgs/Pose2D goal

# if mission is "record_position", set name of position
string position_key
