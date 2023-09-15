#include "ros/ros.h"
#include "robot_interface/fsm_item.h"
#include "robot_interface/fsm.h"
#include "robot_interface/Interface.h"
#include "robot_interface/record_pose.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Char.h"

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
    ros::Subscriber sub_pose_;
    ros::Publisher pub_start_gmapping_;
    ros::Publisher pub_vel_;
    ros::Publisher pub_goal_;

    // topic buffer
    robot_interface::Interface interface_buf_;
    geometry_msgs::Twist navi_vel_buf_;
    geometry_msgs::Twist control_vel_buf_;
    Pose pose_buf_;


    // parameter
    double process_frequency_;
    double velocity_epsilon_;

    ros::Timer timer_;

    // record position
    std::map<std::string, Pose> user_position_dict_;

    FSM *fsm;

    Interface(ros::NodeHandle &nh, ros::NodeHandle &nh_local);

    void initialize();

    void updateState();

    void execute();
    
    void timerCB(const ros::TimerEvent &);

    void interfaceCB(const robot_interface::Interface::ConstPtr& msg);

    void controlVelCB(const geometry_msgs::Twist& msg);

    void naviVelCB(const geometry_msgs::Twist& msg);

    void poseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

    void finishCB(const std_msgs::CharConstPtr& msg);

private:
};