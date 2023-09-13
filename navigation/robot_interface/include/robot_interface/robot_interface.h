#include "ros/ros.h"
#include "robot_interface/fsm_item.h"
#include "robot_interface/fsm.h"

class Interface{
public:
    ros::NodaHandle nh_, nh_local_;
    ros::ServiceServer ser_start_slam_;
    ros::ServiceServer ser_finish_slam_;
    ros::ServiceServer ser_record_;
    ros::Subscriber sub_vel_;
    ros::Subscriber sub_goal_;
    ros::Publisher sub_arrived_;


    FSM *fsm;

    Interface(ros::NodeHandle &nh, ros::NodeHandle &nh_local){
        nh_ = nh;
        nh_local_ = nh_local;
        *fsm = new FSM();
    }

    void initialize();

private:
};