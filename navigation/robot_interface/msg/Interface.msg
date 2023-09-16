# missions:["start_mapping", "finish_control_mapping", "check_map", "move_goal", "move_goal_key", "record_position"]
string mission

# if mission is "move_goal", set goal
geometry_msgs/PoseStamped goal

# if mission is "record_position", set name of position
string position_key

# if mission is "check_map", set map name
bool map_ok
string map_name