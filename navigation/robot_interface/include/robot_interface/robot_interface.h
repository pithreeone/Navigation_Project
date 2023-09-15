#include "ros/ros.h"
#include "robot_interface/fsm_item.h"
#include "robot_interface/fsm.h"
#include "robot_interface/Interface.h"
#include "geometry_msgs/Twist.h"

enum Mission{
    start_mapping,
    finish_control_mapping,
    check_map,
    move_goal,
    record_position,
};

class Interface{
public:
    ros::NodeHandle nh_, nh_local_;
    // from teleop package 
    ros::Subscriber sub_control_vel_;
    // from move_base navigation system
    ros::Subscriber sub_navi_vel_;
    ros::Subscriber sub_action_;
    ros::Subscriber sub_arrived_;
    ros::Publisher pub_start_gmapping_;
    ros::Publisher pub_vel_;

    robot_interface::Interface interface_buf_;
    geometry_msgs::Twist navi_vel_buf_;
    geometry_msgs::Twist control_vel_buf_;

    // parameter
    double process_frequency_;
    double velocity_epsilon_;

    ros::Timer timer_;

    FSM *fsm;

    Interface(ros::NodeHandle &nh, ros::NodeHandle &nh_local);

    void initialize();

    void updateState();

    void execute();
    
    void timerCB(const ros::TimerEvent &);

    void interfaceCB(const robot_interface::Interface::ConstPtr& msg);

    void controlVelCB(const geometry_msgs::Twist& msg);

    void naviVelCB(const geometry_msgs::Twist& msg);

private:
};