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
    nh_local_(nh_local)
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
    pub_start_gmapping_ = nh_.advertise<std_msgs::Bool>("slam_gmapping/reset", 1);
    pub_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("nav_goal", 10);


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
    }else if(interface_buf_.mission.compare("move_goal_key") == 0){
        fsm->handleEvent(FSMItem::Events::E_MOVE_TO_GOAL_KEY);
    }else if(interface_buf_.mission.compare("record_position") == 0){
        fsm->handleEvent(FSMItem::Events::E_RECORD_COORDINATE);
    }else{
        fsm->handleEvent(FSMItem::Events::E_NAN);
    }
    interface_buf_.mission = "";
}

void Interface::execute()
{
    bool if_publish_vel = false;
    // if have not finished mission, just do it!
    if(!fsm->ifFinishMission()){
        switch(fsm->getState()){
            case FSMItem::State::AUTO_MAPPING:{
                std_msgs::Int8 msg;
                msg.data = 2;
                pub_start_gmapping_.publish(msg);
                break;
            }
            case FSMItem::State::RECORD_COORDINATE:{
                user_position_dict_[interface_buf_.position_key] = {pose_buf_.getX(), pose_buf_.getY(), pose_buf_.getTheta()};
                break;
            }
            case FSMItem::State::MOVE_TO_GOAL:{
                geometry_msgs::PoseStamped goal;
                goal = interface_buf_.goal;
                pub_goal_.publish(goal);
                break;
            }
            case FSMItem::State::MOVE_TO_GOAL_KEY:{
                if(user_position_dict_.find(interface_buf_.mission) == user_position_dict_.end()){
                    ROS_INFO("robot_interface: cannot move to goal by key. cannot find position name:%s", interface_buf_.mission.c_str());
                    fsm->handleEvent(FSMItem::Events::E_FINISH_MOVE);
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
    // continue publish velocity
    switch(fsm->getState()){
        case FSMItem::State::AUTO_MAPPING:{
            geometry_msgs::Twist data;
            data = navi_vel_buf_;
            pub_vel_.publish(data);
            if_publish_vel = true;
            break;
        }
        case FSMItem::State::CONTROL_MAPPING:{
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
        if_publish_vel = true;
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
        fsm->handleEvent(FSMItem::Events::E_FINISH_MOVE);
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