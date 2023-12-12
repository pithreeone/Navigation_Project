#include "ros/ros.h"
#include "robot_interface/fsm_item.h"
#include "robot_interface/fsm.h"
#include "pme_amr_msg/Interface.h"
#include "pme_amr_msg/RobotState.h"
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

enum Publish_State{
    REACH,
    STUCK,
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
    ros::Subscriber sub_finish_exploration_;
    ros::Subscriber sub_floor_;
    ros::Subscriber sub_elevator_status_;
    ros::Subscriber sub_elevator_open_status_;
    ros::Publisher pub_start_gmapping_;
    ros::Publisher pub_start_exploration_;
    ros::Publisher pub_vel_;
    ros::Publisher pub_goal_;
    ros::Publisher pub_robot_state_;
    ros::Publisher pub_mechanism_mission_;
    ros::Publisher pub_initial_state_;

    // topic buffer
    pme_amr_msg::Interface interface_buf_;
    geometry_msgs::Twist navi_vel_buf_;
    geometry_msgs::Twist control_vel_buf_;
    Pose pose_buf_;
    int floor_;
    int elevator_status_;
    std::string go_left_or_right_;
    int elevator_open_status_;
    ros::Time t_recent_floor_;
    bool get_floor_;


    std::string cur_map_;


    // parameter
    std::string map_frame_;
    double process_frequency_;
    double publish_velocity_frequency_;
    double linear_velocity_epsilon_;
    double angular_velocity_epsilon_;
    double time_out_t_;
    double resend_frequency_;
    XmlRpc::XmlRpcValue goal_xml_;
    std::vector<std::vector<std::vector<double>>> goal_list_;


    ros::Timer timer_;
    ros::Timer timer_velocity_;

    // record position
    std::map<std::string, Pose> user_position_dict_;

    FSM *fsm;
    FSMItem::Events event;

    Interface(ros::NodeHandle &nh, ros::NodeHandle &nh_local);

    void initialize();

    void updateState();

    void execute();

    void publishState(Publish_State state);

    void checkSubFloor();

    // argument: f->floor, n->number of goal
    void publishGoalFromList(int f, int n);

    // argument: f->floor, n->number of goal
    void publishInitialStateFromList(int f, int n);

    void timerCB(const ros::TimerEvent &);

    void timerVelocityCB(const ros::TimerEvent &);

    void interfaceCB(const pme_amr_msg::Interface::ConstPtr& msg);

    void controlVelCB(const geometry_msgs::Twist::ConstPtr& msg);

    void naviVelCB(const geometry_msgs::Twist& msg);

    void poseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

    void finishCB(const std_msgs::CharConstPtr& msg);

    void finishExplorationCB(const std_msgs::Bool::ConstPtr& msg);

    void floorCB(const std_msgs::Int8::ConstPtr& msg);

    void elevatorCB(const std_msgs::Int8::ConstPtr& msg);

    void elevatorOpenCB(const std_msgs::Int8::ConstPtr& msg);
private:
};