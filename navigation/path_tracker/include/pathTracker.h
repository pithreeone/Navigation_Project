#include <ros/console.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <cmath>
// message
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>
#include <std_srvs/Empty.h>

/** process of pathTracker when each event happen **/

// (1) normal mode
//     goal received                                    reached
// IDLE ----------> GLOBAL_PATH_RECEIVED --> TRACKING ----------> IDLE
// 
// (2) goal changed when tracking
//    goal received                                  goal received              reached
// IDLE ----------> GLOBAL_PATH_RECEIVED --> TRACKING ---------->  TRANSITION ----------> IDLE
// 
// (3) goal is so closed to rival, stopped by nav_main. 
//    goal received                               rival near send -1
// IDLE ----------> GLOBAL_PATH_RECEIVED --> TRACKING ---------->  IDLE
// 
// (4) goal is blocked. fail to get plan
//    cannot get plan 
// IDLE ----------> IDLE
// 
// (5) goal is blocked when tracking
//    goal received                                  something block goal, set local goal to goal
// IDLE ----------> GLOBAL_PATH_RECEIVED --> TRACKING ---------------------------------------->  IDLE


enum class MODE {
    IDLE = 0,
    TRACKING,
    TRANSITION,
    GLOBAL_PATH_RECEIVED
};

enum class VELOCITY {
    LINEAR = 0,
    ANGULAR
};

enum class ODOM_CALLBACK_TYPE {
    nav_msgs_Odometry = 0,
    geometry_msgs_PoseWithCovarianceStamped
};

class RobotState {
   public:
    RobotState(double x, double y, double theta);
    RobotState() {
    }
    double x_;
    double y_;
    double theta_;
    Eigen::Vector3d getVector();
    double distanceTo(RobotState);
};

class PathTracker {
   public:
    PathTracker(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
    ~PathTracker();

    bool initializeParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    void initialize();

   private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    ros::ServiceServer params_srv_;

    // subscriber
    ros::Subscriber pose_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber obstacle_sub_;
    ros::Subscriber action_sub_;
    void Pose_type0_Callback(const nav_msgs::Odometry::ConstPtr& pose_msg);
    void Pose_type1_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg);
    void Goal_Callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

    // void actionCallback(const std_msgs::Bool::ConstPtr& action_msg);

    geometry_msgs::PoseArray obstacle_pose_;
    geometry_msgs::PoseArray rival_pose_;
    ODOM_CALLBACK_TYPE odom_callback_type_;

    // Publisher
    ros::Publisher vel_pub_;
    ros::Publisher local_goal_pub_;
    ros::Publisher pose_array_pub_;
    ros::Publisher goal_reached_pub_;

    std_msgs::Char goal_reached_;

    void Velocity_Publish();

    // Client
    bool Planner_Client(RobotState, RobotState);

    RobotState goal_pose_;
    RobotState cur_pose_;
    RobotState velocity_state_;

    bool is_local_goal_final_reached_;
    bool is_reached_target_range_;

    bool is_XY_Reached(RobotState cur_pose, RobotState goal_pose);
    bool is_Theta_Reached(RobotState cur_pose, RobotState goal_pose);

    // Goal request from Main and Path received from global planner
    std::vector<RobotState> global_path_;
    std::vector<RobotState> global_path_past_;

    // timer setup
    ros::Timer timer_;
    void Timer_Callback(const ros::TimerEvent& e);

    MODE working_mode_;
    MODE working_mode_pre_;

    void Switch_Mode(MODE next_mode);
    bool is_global_path_switched_;

    // controller parameter
    std::string frame_;
    bool p_active_;
    double control_frequency_;
    double lookahead_d_;
    double blocked_lookahead_d_;
    double waiting_timeout_;

    double linear_kp_;
    double linear_max_vel_;
    double linear_acceleration_;
    double linear_brake_distance_;
    double linear_transition_vel_;
    double linear_transition_acc_;
    // double linear_tracking_vel_;
    double xy_tolerance_;
    double linear_min_brake_distance_;
    double linear_brake_distance_ratio_;
    // velocity profile type : linear, smooth_step
    std::string linear_acceleration_profile_;
    std::string linear_deceleration_profile_;

    double angular_kp_;
    double angular_max_vel_;
    double angular_acceleration_;
    double angular_brake_distance_;
    double angular_transition_vel_;
    double angular_transition_acc_;
    double theta_tolerance_;
    double theta_err;
    // velocity profile type : p_control, linear, smooth_step
    std::string angular_acceleration_profile_;
    std::string angular_deceleration_profile_;

    double Angle_Limit_Checking(double theta);
    double Velocity_Profile(VELOCITY, RobotState cur_pos, RobotState goal_pos, RobotState velocity_state, double acceleration_sign);
    double max_linear_vel_reached_;

    // Path post process
    std::vector<RobotState> Orientation_Filter(std::vector<RobotState>);
    int rotate_direction_;

    // rollingWindow method flag
    RobotState Rolling_Window(RobotState cur_pos, std::vector<RobotState> global_path, double R);

    void Omni_Controller(RobotState local_goal, RobotState cur_pos);

    ros::Time t_bef_;
    ros::Time t_now_;
    double dt_;

    // check if goal is blocked after goal received
    bool new_goal;
    bool is_goal_blocked_;
};
