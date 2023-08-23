#include <ros/console.h>
#include <ros/ros.h>
#include <ros/time.h>
#define _USE_MATH_DEFINES
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

enum class MODE {
    MOVE = 0,
    ROTATE,
    VIBRATE,
    MOVEANDROTATE,
    IDLE
};

class DockTracker {
   public:
    DockTracker(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
    ~DockTracker();
    bool initializeParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    void initialize();

   private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    ros::ServiceServer params_srv_;

    // Subscriber
    ros::Subscriber goal_sub_;
    ros::Subscriber pose_sub_;
    // ros::Subscriber rival1_sub_;
    // ros::Subscriber rival2_sub_;
    void goalCB(const geometry_msgs::PoseStamped& data);
    void poseCB_Odometry(const nav_msgs::Odometry& data);
    void poseCB_PoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStamped& data);
    // void rivalCB_Odometry(const nav_msgs::Odometry& data);

    // Publisher
    ros::Publisher pub_;
    ros::Publisher goalreachedPub_;
    void velocityPUB();

    // Timer
    ros::Timer timer_;
    void timerCB(const ros::TimerEvent& e);

    void move();
    void rotate();
    void vibrate();

    double t_bef_;
    double t_now_;

    bool move_finished_;
    bool rotate_finished_;
    // bool rival_appeared_;

    double goal_[3];
    double pose_[3];
    double vel_[3];
    double dock_dist_;
    bool if_get_goal_;
    bool count_dock_dist_;
    double a_;
    double dist_;
    double ang_diff_;
    double cosx_;
    double sinx_;
    // double rival_dist_;

    bool p_active_;
    double control_frequency_;
    double linear_max_vel_;
    double angular_max_vel_;
    double cake_linear_max_vel_;
    double cake_angular_max_vel_;
    double cherry_linear_max_vel_;
    double cherry_angular_max_vel_;
    double div_;
    double profile_percent_;
    double tolerance_;
    double ang_tolerance_;
    int odom_type_;
    // double rival_tolerance_;

    // vibrate-mode
    int vibrate_time_now_;
    int vibrate_time_goal_;
    double vibrate_linear_max_vel_;
    double vibrate_pos_start_x_;
    double vibrate_pos_start_y_;
    double vibrate_tolerance_;
    double vibrate_lin_dist_;

    MODE mode_;
    double distance(double x1, double y1, double x2, double y2);
};