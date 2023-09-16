#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/PoseStamped.h"
#include "robot_interface/fsm_item.h"
#include "robot_interface/fsm.h"
#include "robot_interface/robot_interface.h"
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>


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

    pub_start_gmapping_ = nh_.advertise<std_msgs::Int8>("slam_gmapping/reset", 1);
    pub_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("nav_goal", 10);
    pub_start_exploration_ = nh_.advertise<std_msgs::Int8>("fron_exp_mission", 1);


    // parameter initialize
    nh_local_.param<double>("process_frequency", process_frequency_, 1.0);
    nh_local_.param<double>("publish_velocity_frequency", publish_velocity_frequency_, 10.0);
    nh_local_.param<double>("linear_velocity_epsilon", linear_velocity_epsilon_, 0.1);
    nh_local_.param<double>("angular_velocity_epsilon", angular_velocity_epsilon_, 0.1);

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
    }else if(event == FSMItem::Events::E_FAST_V){
        fsm->handleEvent(event);
    }else if(event == FSMItem::Events::E_SLOW_V){
        fsm->handleEvent(event);
    }else if(event == FSMItem::Events::E_FINISH_MOVE){
        fsm->handleEvent(event);
    }else if(event == FSMItem::Events::E_FINISH_AUTO_MAPPING){
        fsm->handleEvent(event);
    }else{
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
                popen(command, "r");
            }
            case FSMItem::State::RECORD_COORDINATE:
            {
                user_position_dict_[interface_buf_.position_key] = {pose_buf_.getX(), pose_buf_.getY(), pose_buf_.getTheta()};
                break;
            }
            case FSMItem::State::MOVE_TO_GOAL:
            {
                geometry_msgs::PoseStamped goal;
                goal = interface_buf_.goal;
                pub_goal_.publish(goal);
                break;
            }
            case FSMItem::State::MOVE_TO_GOAL_KEY:
            {
                if(user_position_dict_.find(interface_buf_.mission) == user_position_dict_.end()){
                    ROS_INFO("Robot_Interface: cannot move to goal by key. cannot find position name:%s", interface_buf_.mission.c_str());
                    event = FSMItem::Events::E_FINISH_MOVE;
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
        }
        fsm->setFinishMission(true);
    }


}

void Interface::timerCB(const ros::TimerEvent &)
{
    fsm->printState();
    updateState();
    execute();
    std::cout << "debug" <<std::endl;
}

void Interface::timerVelocityCB(const ros::TimerEvent &)
{
    // continue publish velocity
    bool if_publish_vel = false;
    switch(fsm->getState()){
        case FSMItem::State::AUTO_MAPPING:
        case FSMItem::State::MOVE_TO_GOAL:
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

void Interface::interfaceCB(const robot_interface::Interface::ConstPtr & msg)
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
    if(msg->data == 1){
        event = FSMItem::Events::E_FINISH_MOVE;
    }
}

void Interface::finishExplorationCB(const std_msgs::Bool::ConstPtr &msg)
{
    if(msg->data == true){
        ROS_INFO("debug!!!!!!!!!!!!!!"); 
        event = FSMItem::Events::E_FINISH_AUTO_MAPPING;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_interface");
    ros::NodeHandle nh, nh_local("~");

    Interface interface(nh, nh_local);
    
    ros::spin();
    return 0;
}