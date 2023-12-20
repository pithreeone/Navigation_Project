#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "robot_interface/fsm_item.h"
#include "robot_interface/fsm.h"
#include "robot_interface/robot_interface.h"
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>

#define PI 3.14159265358 

Interface::Interface(ros::NodeHandle &nh, ros::NodeHandle &nh_local):
    nh_(nh),
    nh_local_(nh_local),
    event(FSMItem::Events::E_NAN)
{
    fsm = new FSM();
    initialize();
}

void Interface::initialize()
{
    sub_action_ = nh_.subscribe("action", 10, &Interface::interfaceCB, this);
    sub_control_vel_ = nh_.subscribe("control_cmd_vel", 10, &Interface::controlVelCB, this);
    sub_navi_vel_ = nh_.subscribe("navi_cmd_vel", 10, &Interface::naviVelCB, this);
    sub_pose_ = nh_.subscribe("ekf_pose", 10, &Interface::poseCB, this);
    sub_arrived_ = nh_.subscribe("finishornot", 10, &Interface::finishCB, this);
    sub_finish_exploration_ = nh_.subscribe("finish_exploration", 10, &Interface::finishExplorationCB, this);
    sub_floor_ = nh_.subscribe("floor", 10, &Interface::floorCB, this);
    sub_elevator_status_ = nh_.subscribe("elevator_status", 10, &Interface::elevatorCB, this);
    sub_elevator_open_status_ = nh_.subscribe("elevator_open_status", 10, &Interface::elevatorOpenCB, this);

    pub_start_gmapping_ = nh_.advertise<std_msgs::Int8>("slam_gmapping/reset", 1);
    pub_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("nav_goal", 10);
    pub_start_exploration_ = nh_.advertise<std_msgs::Int8>("fron_exp_mission", 1);
    pub_robot_state_ = nh_.advertise<pme_amr_msg::RobotState>("robot_state", 1);
    pub_mechanism_mission_ = nh_.advertise<std_msgs::UInt8MultiArray>("amr_mission", 1);
    pub_initial_state_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

    // parameter initialize
    nh_local_.param<std::string>("map_frame", map_frame_, "map");
    nh_local_.param<double>("process_frequency", process_frequency_, 1.0);
    nh_local_.param<double>("publish_velocity_frequency", publish_velocity_frequency_, 10.0);
    nh_local_.param<double>("linear_velocity_epsilon", linear_velocity_epsilon_, 0.1);
    nh_local_.param<double>("angular_velocity_epsilon", angular_velocity_epsilon_, 0.1);
    nh_local_.param<double>("time_out_t", time_out_t_, 10);
    nh_local_.param<double>("resend_frequency", resend_frequency_, 1);
    nh_local_.getParam("goal", goal_xml_);

    // parse (x, y, yaw) to goal_
    for(int i=0; i<goal_xml_.size(); i++){
        std::vector<std::vector<double>> goal_temp_1;
        for(int j=0; j<goal_xml_[i].size();j++){
            std::vector<double> goal_temp_2;
            for(int k=0; k<goal_xml_[i][j].size(); k++){
                goal_temp_2.push_back(static_cast<double>(goal_xml_[i][j][k]));
            }
            goal_temp_1.push_back(goal_temp_2);
        }
        goal_list_.push_back(goal_temp_1);
    }

    timer_ = nh_.createTimer(ros::Duration(double(1/process_frequency_)), &Interface::timerCB, this);
    timer_velocity_ = nh_.createTimer(ros::Duration(double(1/publish_velocity_frequency_)), &Interface::timerVelocityCB, this);
}   

void Interface::updateState()
{
    // std::cout << interface_buf_.mission << std::endl;
    if(interface_buf_.mission.compare("start_mapping") == 0){
        fsm->handleEvent(FSMItem::Events::E_START_MAPPING);
    }else if(interface_buf_.mission.compare("finish_control_mapping") == 0){
        fsm->handleEvent(FSMItem::Events::E_FINISH_CONTROL_MAPPING);
    }else if(interface_buf_.mission.compare("check_map") == 0){
        fsm->handleEvent(FSMItem::Events::E_FINISH_CHECK_MAP);
    }else if(interface_buf_.mission.compare("choose_map") == 0){
        fsm->handleEvent(FSMItem::Events::E_CHOOSE_MAP);
    }else if(interface_buf_.mission.compare("move_goal") == 0){
        fsm->handleEvent(FSMItem::Events::E_MOVE_TO_GOAL);
    }else if(interface_buf_.mission.compare("move_goal_key") == 0){
        fsm->handleEvent(FSMItem::Events::E_MOVE_TO_GOAL_KEY);
    }else if(interface_buf_.mission.compare("record_position") == 0){
        fsm->handleEvent(FSMItem::Events::E_RECORD_COORDINATE);
    }else if(interface_buf_.mission.compare("move_to_goal_floor") == 0){
        if(get_floor_ == true){
            fsm->handleEvent(FSMItem::Events::E_MOVE_TO_GOAL_FLOOR);
        }else{
            ROS_ERROR("Robot_Interface: Cannot subscribe current floor.");
        }
    }else if(interface_buf_.mission.compare("debug") == 0){
        fsm->handleEvent(FSMItem::Events::E_DEBUG);
    }
    else if(event == FSMItem::Events::E_FAST_V){
        fsm->handleEvent(event);
    }else if(event == FSMItem::Events::E_SLOW_V){
        fsm->handleEvent(event);
    }else if(event == FSMItem::Events::E_FINISH_MOVE_SUCCESS){
        fsm->handleEvent(event);
    }else if(event == FSMItem::Events::E_FINISH_MOVE_FAIL){
        fsm->handleEvent(event);
    }else if(event == FSMItem::Events::E_FINISH_AUTO_MAPPING){
        fsm->handleEvent(event);
    }else if(event == FSMItem::Events::E_DIFFERENT_FLOOR_MOVE){
        fsm->handleEvent(event);
    }else if(event == FSMItem::Events::E_SAME_FLOOR_MOVE){
        fsm->handleEvent(event);
    }else if(event == FSMItem::Events::E_FINISH_RAISE_HAND){
        fsm->handleEvent(event);
    }else if(event == FSMItem::Events::E_GET_DOOR){
        fsm->handleEvent(event);
    }else if(event == FSMItem::Events::E_FINISH_SAY){
        fsm->handleEvent(event);
    }else if(event == FSMItem::Events::E_SUCCESS_UPDOWN){
        fsm->handleEvent(event);
    }else if(event == FSMItem::Events::E_FINISH_SAY){
        fsm->handleEvent(event);
    }else if(event == FSMItem::Events::E_SUCCESS_UPDOWN){
        fsm->handleEvent(event);
    }
    else{
        fsm->handleEvent(FSMItem::Events::E_NAN);
    }
    event = FSMItem::Events::E_NAN;
    interface_buf_.mission = "";
}

void Interface::execute()
{
    // if have not finished mission, just do it!
    if(!fsm->ifFinishMission()){
        switch(fsm->getState()){
            case FSMItem::State::AUTO_MAPPING:
            {
                // start gmapping
                if(fsm->getPreviousState() == FSMItem::State::START_MAPPING){
                    std_msgs::Int8 msg;
                    msg.data = 2;
                    pub_start_gmapping_.publish(msg);
                }

                // start frontier_exploration
                std_msgs::Int8 msg_exp;
                msg_exp.data = 1;
                pub_start_exploration_.publish(msg_exp);
                break;
            }
            case FSMItem::State::SAVE_MAP:
            {
                if(!interface_buf_.map_ok)
                    return;
                std::string str;
                str = "rosrun map_server map_saver -f " + interface_buf_.set_map_name;
                const char *command = str.c_str();
                ROS_INFO("Robot_Interface: save map file using '%s'", command);
                int _ = std::system(command);
            }
            case FSMItem::State::NAVIGATION_MODE:
            {
                cur_map_ = interface_buf_.choose_map_name;
                std::string str;
                str = "rosrun map_server map_server ${MAP_PATH}/" + cur_map_ + ".yaml";
                const char *command = str.c_str();
                ROS_INFO("Robot_Interface: open map file using '%s'", command);
                auto _ = popen(command, "r");
            }
            case FSMItem::State::RECORD_COORDINATE:
            {
                user_position_dict_[interface_buf_.position_key] = {pose_buf_.getX(), pose_buf_.getY(), pose_buf_.getTheta()};
                break;
            }
            case FSMItem::State::MOVE_TO_GOAL:
            {
                pub_goal_.publish(interface_buf_.goal);
                break;
            }
            case FSMItem::State::MOVE_TO_GOAL_KEY:
            {
                if(user_position_dict_.find(interface_buf_.mission) == user_position_dict_.end()){
                    ROS_INFO("Robot_Interface: cannot move to goal by key. cannot find position name:%s", interface_buf_.mission.c_str());
                    event = FSMItem::Events::E_FINISH_MOVE_FAIL;
                    break;
                }
                geometry_msgs::PoseStamped goal;
                double yaw;
                goal.pose.position.x = user_position_dict_[interface_buf_.mission].getX();
                goal.pose.position.y = user_position_dict_[interface_buf_.mission].getY();
                yaw = user_position_dict_[interface_buf_.mission].getTheta();
                tf::Quaternion q;
                q.setRPY(0, 0, yaw);
                goal.pose.orientation.x = q.x();
                goal.pose.orientation.y = q.y();
                goal.pose.orientation.z = q.z();
                goal.pose.orientation.w = q.w();
                pub_goal_.publish(goal);           
                break;
            }
            case FSMItem::State::TEMP_STOP:
            {
                // convert int -> string
                std::stringstream ss;
                ss << floor_;
                std::string str = "rosrun map_server map_server ${MAP_PATH}/EngBuild" + ss.str() + ".yaml";
                const char *command = str.c_str();
                ROS_INFO("Robot_Interface: open map file using '%s'", command);
                auto _ = popen(command, "r");

                // check whether the floor of goal is same as now
                if(static_cast<int>(interface_buf_.floor.data) != floor_){
                    event = FSMItem::Events::E_DIFFERENT_FLOOR_MOVE;
                }else{
                    event = FSMItem::Events::E_SAME_FLOOR_MOVE;
                }
                break;
            }
            case FSMItem::State::MOVE_TO_GOAL_1:
            {
                publishGoalFromList(floor_, 1);
                break;
            }
            case FSMItem::State::RAISE_HAND:
            {
                static int time = 0;
                if(time == 0){
                    std_msgs::UInt8MultiArray msg;
                    msg.data.push_back(1);
                    msg.data.push_back(1);
                    pub_mechanism_mission_.publish(msg);
                    time++;
                    return;
                }else if(time < 2 * process_frequency_){
                    time++;
                    return;
                }else{
                    event = FSMItem::Events::E_FINISH_RAISE_HAND;
                    time = 0;
                    return;
                }
                break;
            }
            case FSMItem::State::MOVE_TO_GOAL_2:
            {
                publishGoalFromList(floor_, 2);
                break;
            }
            case FSMItem::State::GET_DOOR:
            {
                if(elevator_status_ != 0){
                    if(elevator_status_ == 1){
                        go_left_or_right_ = "right";
                        ROS_INFO("Robot_Interface: Go right!!");
                    }else if(elevator_status_ == 2){
                        go_left_or_right_ = "left";
                        ROS_INFO("Robot_Interface: Go left!!");
                    }
                    event = FSMItem::Events::E_GET_DOOR;
                }
                return;
                break;
            }
            case FSMItem::State::MOVE_TO_GOAL_3:
            {
                ROS_INFO("elevator_status: %d", elevator_status_);
                if(go_left_or_right_.compare("right") == 0){
                    publishGoalFromList(floor_, 4);
                }else if(go_left_or_right_.compare("left") == 0){
                    publishGoalFromList(floor_, 3);
                }
                
                break;
            }
            case FSMItem::State::MOVE_INTO_ELEVATOR:
            {
                if(go_left_or_right_.compare("right") == 0){
                    publishGoalFromList(floor_, 6);
                }else if(go_left_or_right_.compare("left") == 0){
                    publishGoalFromList(floor_, 5);
                }
                // publishGoalFromList(floor_, 5);
                ROS_ERROR("NOT IMPLEMENT ERROR");
                break;
            }
            case FSMItem::State::SAY_FLOOR:
            {
                static int time = 0;
                if(time == 0){
                    std::string floor = std::to_string(static_cast<int>(interface_buf_.floor.data));
                    ROS_INFO("floor:%s", floor.c_str());
                    std::string str = "mpg321 ${MUSIC_PATH}/f1/" + floor + "f.mp3";
                    const char *command1 = str.c_str();
                    auto _ = popen(command1, "r");
                    time++;
                    return;
                }else if(time < 4 * process_frequency_){
                    time++;
                    return;
                }else{
                    std::string str = "mpg321 ${MUSIC_PATH}/f1/close-the-door.mp3";
                    const char *command2 = str.c_str();
                    auto _ = popen(command2, "r");
                    event = FSMItem::Events::E_FINISH_SAY;
                    return;
                }
                
                break;
            }
            case FSMItem::State::WAIT_FOR_ELEVATOR:
            {
                if(floor_ == interface_buf_.floor.data){
                    if(elevator_open_status_ == 1){
                        event = FSMItem::Events::E_SUCCESS_UPDOWN;
                    }
                }
                
                static bool publish_map = false;
                if(publish_map == false){
                    std::stringstream ss;
                    ss << static_cast<int>(interface_buf_.floor.data);
                    std::string str = "rosrun map_server map_server ${MAP_PATH}/EngBuild" + ss.str() + ".yaml";
                    const char *command = str.c_str();
                    ROS_INFO("Robot_Interface: open map file using '%s'", command);
                    auto _ = popen(command, "r");
                    if(go_left_or_right_.compare("right") == 0){
                        publishInitialStateFromList(interface_buf_.floor.data, 4);
                    }else if(go_left_or_right_.compare("left") == 0){
                        publishInitialStateFromList(interface_buf_.floor.data, 3);
                    }
                    publish_map = true;
                }

                return;
                break;
            }
            case FSMItem::State::GET_OUT_OF_ELEVATOR:
            {
                int floor_now = interface_buf_.floor.data;
                if(go_left_or_right_.compare("right") == 0){
                    publishGoalFromList(floor_now, 6);
                }else if(go_left_or_right_.compare("left") == 0){
                    publishGoalFromList(floor_now, 5);
                }
                break;
            }
            case FSMItem::State::MOVE_TO_GOAL_4:
            {
                pub_goal_.publish(interface_buf_.goal);
                break;
            }
        }
        fsm->setFinishMission(true);
    }


}

void Interface::publishState(Publish_State state)
{
    pme_amr_msg::RobotState msg;
    msg.robot_id.data = 0;
    switch(state){
        case Publish_State::REACH:{
            msg.state.data = "GOAL";
            break;
        }
        case Publish_State::STUCK:{
            msg.state.data = "STUCK";
            break;
        }
    }
    pub_robot_state_.publish(msg);
}

void Interface::checkSubFloor()
{
    get_floor_ = true;
    if((ros::Time::now() - t_recent_floor_).toSec() > 0.1){
        ROS_WARN_THROTTLE(0.5, "Robot_Interface: Cannot subscribe current floor ... ");
        get_floor_ = false;
    }
}

void Interface::publishGoalFromList(int f, int n)
{
    // publish the n'th goal in goal_list
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = map_frame_;
    goal.pose.position.x = goal_list_[f-1][n-1][0];
    goal.pose.position.y = goal_list_[f-1][n-1][1];
    tf::Quaternion q;
    q.setRPY(0, 0, goal_list_[f-1][n-1][2]);
    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(q, odom_quat);
    goal.pose.orientation = odom_quat;
    pub_goal_.publish(goal);
}

void Interface::publishInitialStateFromList(int f, int n)
{
    // publish the n'th goal in goal_list
    geometry_msgs::PoseWithCovarianceStamped state;
    state.header.stamp = ros::Time::now();
    state.header.frame_id = map_frame_;
    state.pose.pose.position.x = goal_list_[f-1][n-1][0];
    state.pose.pose.position.y = goal_list_[f-1][n-1][1];
    tf::Quaternion q;
    q.setRPY(0, 0, goal_list_[f-1][n-1][2]);
    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(q, odom_quat);
    state.pose.pose.orientation = odom_quat;

    // state.pose.covariance[0 * 6 + 0] = 1;
    // state.pose.covariance[1 * 6 + 1] = 1;
    // state.pose.covariance[5 * 6 + 6] = 1;

    pub_initial_state_.publish(state);
}

void Interface::timerCB(const ros::TimerEvent &)
{
    fsm->printState();
    checkSubFloor();
    updateState();
    execute();
}

void Interface::timerVelocityCB(const ros::TimerEvent &)
{
    // continue publish velocity
    bool if_publish_vel = false;
    switch(fsm->getState()){
        case FSMItem::State::AUTO_MAPPING:
        case FSMItem::State::MOVE_TO_GOAL:
        case FSMItem::State::MOVE_TO_GOAL_1:
        case FSMItem::State::MOVE_TO_GOAL_2:
        case FSMItem::State::MOVE_TO_GOAL_3:
        case FSMItem::State::MOVE_INTO_ELEVATOR:
        case FSMItem::State::MOVE_TO_GOAL_4:
        case FSMItem::State::GET_OUT_OF_ELEVATOR:
        {
            geometry_msgs::Twist data;
            data = navi_vel_buf_;
            pub_vel_.publish(data);
            if_publish_vel = true;
            break;
        }
        case FSMItem::State::CONTROL_MAPPING:
        case FSMItem::State::CONTROL_MOVING:
        {
            geometry_msgs::Twist data;
            data = control_vel_buf_;
            pub_vel_.publish(data);
            if_publish_vel = true;
            break;
        }
    }

    if(!if_publish_vel){
        geometry_msgs::Twist data;
        data.linear.x = data.linear.y = data.angular.z = 0.0;
        pub_vel_.publish(data);
        if_publish_vel = true;
    }
}

void Interface::interfaceCB(const pme_amr_msg::Interface::ConstPtr & msg)
{
    interface_buf_ = *msg;
}

void Interface::controlVelCB(const geometry_msgs::Twist::ConstPtr& msg)
{
    control_vel_buf_ = *msg;
    double linear_v = sqrt(pow(msg->linear.x, 2) + pow(msg->linear.y, 2));
    double angular_v = fabs(msg->angular.z);
    if(linear_v > linear_velocity_epsilon_ || angular_v > angular_velocity_epsilon_){
        event = FSMItem::Events::E_FAST_V;
    }else{
        event = FSMItem::Events::E_SLOW_V;
    }
}

void Interface::naviVelCB(const geometry_msgs::Twist &msg)
{
    navi_vel_buf_ = msg;

}

void Interface::poseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    tf::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    pose_buf_.setPose(x, y, yaw);
}

void Interface::finishCB(const std_msgs::CharConstPtr &msg)
{
    // if the current state is not MOVE_TO_GOAL, just skip !!
    if(fsm->getState() != FSMItem::State::MOVE_TO_GOAL &&
       fsm->getState() != FSMItem::State::MOVE_TO_GOAL_1 &&
       fsm->getState() != FSMItem::State::MOVE_TO_GOAL_2 &&
       fsm->getState() != FSMItem::State::MOVE_TO_GOAL_3 &&
       fsm->getState() != FSMItem::State::MOVE_INTO_ELEVATOR &&
       fsm->getState() != FSMItem::State::MOVE_TO_GOAL_4 &&
       fsm->getState() != FSMItem::State::GET_OUT_OF_ELEVATOR)
        return;

    if(msg->data == 1){
        event = FSMItem::Events::E_FINISH_MOVE_SUCCESS;
        publishState(Publish_State::REACH);
    }
    // when pathTracker return (2, not OK), resend goal.
    else if(msg->data == 2){
        static int resend_n = 1;
        int resend_n_max = time_out_t_ * process_frequency_;
        if(++resend_n <= resend_n_max){
            fsm->setFinishMission(false);
            ROS_INFO("Robot_Interface: resend goal! times:%d", resend_n);
        }else{
            event = FSMItem::Events::E_FINISH_MOVE_FAIL;
            resend_n = 1;
            publishState(Publish_State::STUCK);
        }
    }
}

void Interface::finishExplorationCB(const std_msgs::Bool::ConstPtr &msg)
{
    if(msg->data == true){
        ROS_INFO("debug!!!!!!!!!!!!!!"); 
        event = FSMItem::Events::E_FINISH_AUTO_MAPPING;
    }
}

void Interface::floorCB(const std_msgs::Int8::ConstPtr &msg)
{
    floor_ = msg->data;
    t_recent_floor_ = ros::Time::now();
}

void Interface::elevatorCB(const std_msgs::Int8::ConstPtr & msg)
{
    if(fsm->getState() == FSMItem::State::GET_DOOR){
        elevator_status_ = static_cast<int>(msg->data);
    }
    
}

void Interface::elevatorOpenCB(const std_msgs::Int8::ConstPtr & msg)
{
    elevator_open_status_ = msg->data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_interface");
    ros::NodeHandle nh, nh_local("~");

    Interface interface(nh, nh_local);
    
    ros::spin();
    return 0;
}