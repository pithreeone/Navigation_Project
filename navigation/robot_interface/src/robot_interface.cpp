#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "robot_interface/fsm_item.h"
#include "robot_interface/fsm.h"
#include "robot_interface/robot_interface.h"


Interface::Interface(ros::NodeHandle &nh, ros::NodeHandle &nh_local):
    nh_(nh),
    nh_local_(nh_local)
{
    fsm = new FSM();
    initialize();
}

void Interface::initialize()
{
    sub_action_ = nh_.subscribe("action", 10, &Interface::interfaceCB, this);
    sub_control_vel_ = nh_.subscribe("control_cmd_vel_", 10, &Interface::controlVelCB, this);
    sub_navi_vel_ = nh_.subscribe("navi_cmd_vel_", 10, &Interface::naviVelCB, this);
    pub_start_gmapping_ = nh_.advertise<std_msgs::Bool>("slam_gmapping/reset", 1);
    pub_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);


    // parameter initialize
    nh_local_.param<double>("process_frequency", process_frequency_, 1);
    nh_local_.param<double>("velocity_epsilon", velocity_epsilon_, 0.1);

    timer_ = nh_.createTimer(ros::Duration(double(1/process_frequency_)), &Interface::timerCB, this);
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
    }else if(interface_buf_.mission.compare("move_goal") == 0){
        fsm->handleEvent(FSMItem::Events::E_MOVE_TO_GOAL);
    }else if(interface_buf_.mission.compare("record_position") == 0){
        fsm->handleEvent(FSMItem::Events::E_RECORD_COORDINATE);
    }else{
        fsm->handleEvent(FSMItem::Events::E_NAN);
    }
    interface_buf_.mission = "";
}

void Interface::execute()
{
    // if have not finished mission, just do it!
    if(!fsm->ifFinishMission()){
        switch(fsm->getState()){
            case FSMItem::State::START_MAPPING:{
                std_msgs::Bool msg;
                msg.data = true;
                pub_start_gmapping_.publish(msg);
                break;
            }
            // case FSMItem::State::
        }
        fsm->setFinishMission(true);
    }
    // continue publish velocity
    switch(fsm->getState()){
        case FSMItem::State::AUTO_MAPPING:{
            geometry_msgs::Twist data;
            data = navi_vel_buf_;
            pub_vel_.publish(data);
            break;
        }
        case FSMItem::State::CONTROL_MAPPING:{
            geometry_msgs::Twist data;
            data = control_vel_buf_;
            pub_vel_.publish(data);
            break;
        }
    }

}

void Interface::timerCB(const ros::TimerEvent &)
{
    fsm->printState();
    updateState();
    execute();


}

void Interface::interfaceCB(const robot_interface::Interface::ConstPtr & msg)
{
    interface_buf_ = *msg;
}

void Interface::controlVelCB(const geometry_msgs::Twist& msg)
{
    control_vel_buf_ = msg;
    double velocity = sqrt(pow(msg.linear.x, 2) + pow(msg.linear.y, 2));
    if(velocity > velocity_epsilon_){
        fsm->handleEvent(FSMItem::Events::E_FAST_V);
    }else if(velocity <= velocity_epsilon_){
        fsm->handleEvent(FSMItem::Events::E_SLOW_V);
    }
}

void Interface::naviVelCB(const geometry_msgs::Twist & msg)
{
    navi_vel_buf_ = msg;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_interface");
    ros::NodeHandle nh, nh_local("~");

    Interface interface(nh, nh_local);
    
    ros::spin();
    return 0;
}