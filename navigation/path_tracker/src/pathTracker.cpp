#include "pathTracker.h"

RobotState::RobotState(double x, double y, double theta) {
    x_ = x;
    y_ = y;
    theta_ = theta;
}

double RobotState::distanceTo(RobotState pos) {
    return sqrt(pow(x_ - pos.x_, 2) + pow(y_ - pos.y_, 2));
}

Eigen::Vector3d RobotState::getVector() {
    Eigen::Vector3d vec;
    vec << x_, y_, theta_;
    return vec;
}

PathTracker::PathTracker(ros::NodeHandle& nh, ros::NodeHandle& nh_local) {
    nh_ = nh;
    nh_local_ = nh_local;
    std_srvs::Empty empt;
    p_active_ = false;
    params_srv_ = nh_local_.advertiseService("params", &PathTracker::initializeParams, this);
    initializeParams(empt.request, empt.response);
    initialize();
    t_bef_ = ros::Time::now();
    t_now_ = ros::Time::now();
}

PathTracker::~PathTracker() {
    nh_local_.deleteParam("active");
    nh_local_.deleteParam("control_frequency");
    nh_local_.deleteParam("lookahead_distance");

    nh_local_.deleteParam("linear_kp");
    nh_local_.deleteParam("linear_max_velocity");
    nh_local_.deleteParam("linear_acceleration");
    nh_local_.deleteParam("linear_brake_distance_ratio");
    nh_local_.deleteParam("linear_min_brake_distance");
    nh_local_.deleteParam("xy_tolerance");
    nh_local_.deleteParam("linear_transition_vel_");
    nh_local_.deleteParam("linear_transition_acc_");
    nh_local_.deleteParam("linear_acceleration_profile");
    nh_local_.deleteParam("linear_deceleration_profile");

    nh_local_.deleteParam("angular_kp");
    nh_local_.deleteParam("angular_max_velocity");
    nh_local_.deleteParam("angular_acceleration");
    nh_local_.deleteParam("angular_brake_distance");
    nh_local_.deleteParam("theta_tolerance");
    nh_local_.deleteParam("angular_transition_vel_");
    nh_local_.deleteParam("angular_transition_acc_");
    nh_local_.deleteParam("angular_acceleration_profile");
    nh_local_.deleteParam("angular_deceleration_profile");
}

void PathTracker::initialize() {
    is_local_goal_final_reached_ = false;
    is_global_path_switched_ = false;
    is_goal_blocked_ = false;
    timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency_), &PathTracker::Timer_Callback, this, false);

    working_mode_ = MODE::IDLE;
    working_mode_pre_ = MODE::IDLE;
}

bool PathTracker::initializeParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    // load parameter
    bool prev_active = p_active_;
    int temp_odom_callback_type_;
    std::string odom_topic_name_;

    nh_local_.param<bool>("active", p_active_, true);
    nh_local_.param<std::string>("map_frame", map_frame_, "map");
    nh_local_.param<std::string>("odom_frame", odom_frame_, "odom");
    nh_local_.param<std::string>("base_frame", base_frame_, "base_footprint");
    nh_local_.param<double>("control_frequency", control_frequency_, 50);
    nh_local_.param<double>("planner_frequency", planner_frequency_, 50);
    nh_local_.param<double>("lookahead_distance", lookahead_d_, 0.2);
    nh_local_.param<double>("waiting_timeout", waiting_timeout_, 3);
    nh_local_.param<double>("blocked_lookahead_distance", blocked_lookahead_d_, 0.2);

    nh_local_.param<int>("odom_type", temp_odom_callback_type_, 0);
    nh_local_.param<std::string>("odom_topic_name", odom_topic_name_, "odom");
    if (temp_odom_callback_type_ == 0) {
        odom_callback_type_ = ODOM_CALLBACK_TYPE::nav_msgs_Odometry;
    } else {
        odom_callback_type_ = ODOM_CALLBACK_TYPE::geometry_msgs_PoseWithCovarianceStamped;
    }

    // linear parameter
    // acceleration
    nh_local_.param<double>("linear_max_velocity", linear_max_vel_, 0.5);
    nh_local_.param<double>("linear_acceleration", linear_acceleration_, 0.3);
    nh_local_.param<std::string>("linear_acceleration_profile", linear_acceleration_profile_, "linear");
    // transition
    nh_local_.param<double>("linear_transition_velocity", linear_transition_vel_, 0.15);
    nh_local_.param<double>("linear_transition_acceleration", linear_transition_acc_, 0.6);
    // deceleration
    nh_local_.param<double>("linear_kp", linear_kp_, 0.8);
    nh_local_.param<double>("linear_brake_distance_ratio", linear_brake_distance_ratio_, 0.3);
    nh_local_.param<double>("linear_min_brake_distance", linear_min_brake_distance_, 0.3);
    nh_local_.param<std::string>("linear_deceleration_profile", linear_deceleration_profile_, "linear");

    // angular parameter
    nh_local_.param<double>("angular_max_velocity", angular_max_vel_, 3);
    nh_local_.param<double>("angular_acceleration", angular_acceleration_, 0.5);
    nh_local_.param<double>("angular_brake_distance", angular_brake_distance_, 0.35);
    nh_local_.param<double>("angular_transition_velocity", angular_transition_vel_, 0.15);
    nh_local_.param<double>("angular_transition_acceleration", angular_transition_acc_, 0.6);
    nh_local_.param<double>("angular_kp", angular_kp_, 1.5);
    nh_local_.param<std::string>("angular_acceleration_profile", angular_acceleration_profile_, "linear");
    nh_local_.param<std::string>("angular_deceleration_profile", angular_deceleration_profile_, "linear");

    nh_local_.param<double>("xy_tolerance", xy_tolerance_, 0.01);
    nh_local_.param<double>("theta_tolerance", theta_tolerance_, 0.03);

    if (p_active_ != prev_active) {
        if (p_active_) {
            if (odom_callback_type_ == ODOM_CALLBACK_TYPE::nav_msgs_Odometry) {
                pose_sub_ = nh_.subscribe(odom_topic_name_, 5, &PathTracker::Pose_type0_Callback, this);
            } else {
                pose_sub_ = nh_.subscribe(odom_topic_name_, 5, &PathTracker::Pose_type1_Callback, this);
            }
            goal_sub_ = nh_.subscribe("nav_goal", 5, &PathTracker::Goal_Callback, this);
            vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
            // local_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("local_goal", 10);
            // pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("orientation", 10);
            goal_reached_pub_ = nh_.advertise<std_msgs::Char>("finishornot", 1);
            pub_mechanism_mission_ = nh_.advertise<std_msgs::UInt8MultiArray>("amr_mission", 1);
        } else {
            pose_sub_.shutdown();
            goal_sub_.shutdown();
            vel_pub_.shutdown();
            // local_goal_pub_.shutdown();
            // pose_array_pub_.shutdown();
            goal_reached_pub_.shutdown();
        }
    }

    ROS_INFO_STREAM("[PathTracker]: set param ok");
    return true;
}

void PathTracker::Timer_Callback(const ros::TimerEvent& e) {
    // ROS_INFO("[PathTracker]: working_mode:%d", working_mode_);
    switch (working_mode_) {
        case MODE::GLOBAL_PATH_RECEIVED: {
            if (working_mode_pre_ == MODE::IDLE || working_mode_pre_ == MODE::GLOBAL_PATH_RECEIVED) {
                Switch_Mode(MODE::TRACKING);
                break;
            } else if (working_mode_pre_ == MODE::TRACKING) {
                // Slow down first then start tracking new path
                Switch_Mode(MODE::TRANSITION);
                break;
            } else if (working_mode_pre_ == MODE::TRANSITION) {
                Switch_Mode(MODE::TRACKING);
                break;
            }
        } break;

        case MODE::TRACKING: {
            // goal reached
            if (is_XY_Reached(cur_pose_, goal_pose_) && is_Theta_Reached(cur_pose_, goal_pose_)) {
                if(!is_goal_blocked_){
                    ROS_INFO("[PathTracker]: GOAL REACHED !");
                    Switch_Mode(MODE::IDLE);
                    velocity_state_.x_ = 0;
                    velocity_state_.y_ = 0;
                    velocity_state_.theta_ = 0;
                    Velocity_Publish();

                    is_goal_blocked_ = false;

                    // publish /finishornot
                    goal_reached_.data = 1;
                    goal_reached_pub_.publish(goal_reached_);
                    break;
                }else if(is_goal_blocked_){
                    ROS_INFO("[PathTracker]: Goal is blocked ... Local goal reached !");
                    Switch_Mode(MODE::IDLE);
                    velocity_state_.x_ = 0;
                    velocity_state_.y_ = 0;
                    velocity_state_.theta_ = 0;
                    Velocity_Publish();

                    is_goal_blocked_ = false;

                    // publish /finishornot
                    goal_reached_.data = 2;
                    goal_reached_pub_.publish(goal_reached_);
                }
            }

            if (working_mode_pre_ == MODE::TRANSITION) {
                if (is_global_path_switched_ == false) {
                    is_local_goal_final_reached_ = false;
                    // Planner_Client(cur_pose_, goal_pose_);
                    linear_brake_distance_ = linear_brake_distance_ratio_ * cur_pose_.distanceTo(goal_pose_);
                    is_global_path_switched_ = true;
                }
            }

            // if goal is blocked, use next local goal as the goal pose to track
            RobotState local_goal;
            if (!Planner_Client(cur_pose_, goal_pose_)) {
                local_goal = Rolling_Window(cur_pose_, global_path_, blocked_lookahead_d_);
                goal_pose_.x_ = local_goal.x_;
                goal_pose_.y_ = local_goal.y_;
                goal_pose_.theta_ = local_goal.theta_;

                global_path_past_ = global_path_;

                is_global_path_switched_ = false;
                is_goal_blocked_ = true;

                return;
            }
            local_goal = Rolling_Window(cur_pose_, global_path_, lookahead_d_);
            Omni_Controller(local_goal, cur_pose_);
        } break;

        case MODE::IDLE: {
            // ROS_INFO("Working Mode : IDLE");
            velocity_state_.x_ = 0;
            velocity_state_.y_ = 0;
            velocity_state_.theta_ = 0;
            Velocity_Publish();
        } break;

        case MODE::TRANSITION: {
            ROS_INFO("Working Mode : TRANSITION");
            double linear_vel = sqrt(pow(velocity_state_.x_, 2) + pow(velocity_state_.y_, 2));
            double angular_vel = velocity_state_.theta_;

            if (linear_vel <= linear_transition_vel_ && angular_vel <= angular_transition_vel_) {
                Switch_Mode(MODE::TRACKING);
                break;
            }

            RobotState local_goal;
            // if the new goal has no path to reach, just change the goal to the local_goal which belong to
            // past_path
            if (!Planner_Client(cur_pose_, goal_pose_)) {
                local_goal = Rolling_Window(cur_pose_, global_path_, lookahead_d_);
                goal_pose_.x_ = local_goal.x_;
                goal_pose_.y_ = local_goal.y_;
                goal_pose_.theta_ = local_goal.theta_;

                global_path_past_ = global_path_;

                Switch_Mode(MODE::TRACKING);

                is_global_path_switched_ = false;
                is_goal_blocked_ = true;

                // publish /finishornot
                goal_reached_.data = 2;
                goal_reached_pub_.publish(goal_reached_);

                return;
            }
            local_goal = Rolling_Window(cur_pose_, global_path_past_, lookahead_d_);
            Omni_Controller(local_goal, cur_pose_);

        } break;
    }
}

void PathTracker::Switch_Mode(MODE next_mode) {
    working_mode_pre_ = working_mode_;
    working_mode_ = next_mode;
}

bool PathTracker::Planner_Client(RobotState cur_pos, RobotState goal_pos) {
    if((ros::Time::now() - time_bef).toSec() < (1.0 / planner_frequency_)){
        return true;
    }
        
    time_bef = ros::Time::now();
    geometry_msgs::PoseStamped cur;
    cur.header.frame_id = map_frame_;
    cur.pose.position.x = cur_pos.x_;
    cur.pose.position.y = cur_pos.y_;
    cur.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, cur_pos.theta_);
    cur.pose.orientation.x = q.x();
    cur.pose.orientation.y = q.y();
    cur.pose.orientation.z = q.z();
    cur.pose.orientation.w = q.w();

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = map_frame_;
    goal.pose.position.x = goal_pos.x_;
    goal.pose.position.y = goal_pos.y_;
    goal.pose.position.z = 0;

    // tf2::Quaternion q;
    q.setRPY(0, 0, goal_pos.theta_);
    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();

    ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetPlan>("move_base/GlobalPlanner/make_plan");
    nav_msgs::GetPlan srv;
    srv.request.start = cur;
    srv.request.goal = goal;

    std::vector<geometry_msgs::PoseStamped> path_msg;

    if (client.call(srv)) {
        new_goal = false;
        if (srv.response.plan.poses.empty()) {
            ROS_WARN_STREAM("[PathTracker] : Got empty plan");
            return false;
        }

        nav_msgs::Path path_msg;
        path_msg.poses = srv.response.plan.poses;

        global_path_.clear();

        for (const auto& point : path_msg.poses) {
            RobotState pose;
            pose.x_ = point.pose.position.x;
            pose.y_ = point.pose.position.y;
            tf2::Quaternion q;
            tf2::fromMsg(point.pose.orientation, q);
            tf2::Matrix3x3 qt(q);
            double _, yaw;
            qt.getRPY(_, _, yaw);
            pose.theta_ = yaw;
            global_path_.push_back(pose);
        }
        global_path_ = Orientation_Filter(global_path_);
        return true;
    } else {
        ROS_ERROR_STREAM("[PathTracker] : Failed to call service make_plan");
        return false;
    }
}

void PathTracker::Pose_type0_Callback(const nav_msgs::Odometry::ConstPtr& pose_msg) {

    geometry_msgs::PoseStamped pose_in_odom, pose_in_map;
    pose_in_odom.header.frame_id = odom_frame_;
    pose_in_odom.header.stamp = ros::Time::now();
    pose_in_odom.pose.position.x = pose_msg->pose.pose.position.x;
    pose_in_odom.pose.position.y = pose_msg->pose.pose.position.y;
    pose_in_odom.pose.position.z = 0;
    pose_in_odom.pose.orientation = pose_msg->pose.pose.orientation;
    
    // get transform from odom to map

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped odom_to_map_tf;

    try{
        // if(tfBuffer.canTransform(map_frame_, "base_footprint", ros::Time(0), ros::Duration(1.0))){
        //     odom_to_map_tf = tfBuffer.lookupTransform(map_frame_, "base_footprint", ros::Time(0));
        //     ROS_INFO("transform successfully");
        // }
        // std::cout << "time1 " << ros::Time::now() << std::endl;
        if(listener.waitForTransform(map_frame_, base_frame_, ros::Time(0), ros::Duration(1.0))){
            // std::cout << "time2 " << ros::Time::now() << std::endl;
            listener.lookupTransform(map_frame_, base_frame_, ros::Time(0), transform);
            // ROS_INFO("transform successfully");
        }
        
    }
    catch(tf::TransformException &ex){
        ROS_WARN("%s", ex.what());
    }
    // ROS_INFO("tf: x:%f, y:%f, z:%f", odom_to_map_tf.transform.translation.x, odom_to_map_tf.transform.translation.y, odom_to_map_tf.transform.translation.z);
    // transform from odom to map
    // tf2::doTransform(pose_in_odom, pose_in_map, odom_to_map_tf);
    // ROS_INFO("pose odom: x:%f, y:%f, map: x:%f, y:%f", pose_in_odom.pose.position.x, pose_in_odom.pose.position.y, pose_in_map.pose.position.x, pose_in_map.pose.position.y);
    cur_pose_.x_ = transform.getOrigin().x();
    cur_pose_.y_ = transform.getOrigin().y();
    tf2::Quaternion q;
    q.setX(transform.getRotation().x());
    q.setY(transform.getRotation().y());
    q.setZ(transform.getRotation().z());
    q.setW(transform.getRotation().w());
    // tf2::fromMsg(transform.getRotation(), q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    cur_pose_.theta_ = yaw;
}

void PathTracker::Pose_type1_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg) {
    cur_pose_.x_ = pose_msg->pose.pose.position.x;
    cur_pose_.y_ = pose_msg->pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(pose_msg->pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    cur_pose_.theta_ = yaw;
}

void PathTracker::Goal_Callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
    ros::ServiceClient client2 = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    std_srvs::Empty srv;
    t_bef_ = ros::Time::now();

    goal_pose_.x_ = pose_msg->pose.position.x;
    goal_pose_.y_ = pose_msg->pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(pose_msg->pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);

    goal_pose_.theta_ = yaw;

    // If the goal is so closed to rival, or other reason that nav_main stopped robot. It will send vx=vy=0
    if(goal_pose_.x_ == -1 && goal_pose_.y_ == -1){
        ROS_INFO("[PathTracker]: Something wrong ! stopped by nav_main");
        Switch_Mode(MODE::IDLE);
        velocity_state_.x_ = 0;
        velocity_state_.y_ = 0;
        velocity_state_.theta_ = 0;
        Velocity_Publish();
        return;
    }

    ROS_INFO("[PathTracker]: Goal received ! (%f, %f, %f)", goal_pose_.x_, goal_pose_.y_, goal_pose_.theta_);

    if (goal_pose_.x_ == -1 && goal_pose_.y_ == -1) {
        Switch_Mode(MODE::IDLE);
        return;
    }

    global_path_past_ = global_path_;

    // if working_mode is IDLE, don't need to deceleration, just send 2 to nav_main
    if (!Planner_Client(cur_pose_, goal_pose_) && working_mode_ == MODE::IDLE) {
        Switch_Mode(MODE::IDLE);
        is_goal_blocked_ = true;
        goal_reached_.data = 2;
        goal_reached_pub_.publish(goal_reached_);
        return;
    }
    is_goal_blocked_ = false;

    // publish /finishornot
    goal_reached_.data = 0;
    goal_reached_pub_.publish(goal_reached_);

    linear_brake_distance_ = linear_brake_distance_ratio_ * cur_pose_.distanceTo(goal_pose_);
    if (linear_brake_distance_ < linear_min_brake_distance_)
        linear_brake_distance_ = linear_min_brake_distance_;

    is_local_goal_final_reached_ = false;
    is_global_path_switched_ = false;
    Switch_Mode(MODE::GLOBAL_PATH_RECEIVED);
    new_goal = true;
}

RobotState PathTracker::Rolling_Window(RobotState cur_pos, std::vector<RobotState> path, double L_d) {
    int k = 1;
    int last_k = 0;
    int d_k = 0;
    RobotState a;
    int a_idx = 0;
    RobotState b;
    int b_idx = 0;
    RobotState local_goal;
    bool if_b_asigned = false;
    double r = L_d;

    // ROS_INFO("%ld", path.size());
    for (int i = 0; i < path.size(); i++) {
        if (i == 1)
            last_k = 0;
        last_k = k;
        if (cur_pos.distanceTo(path.at(i)) >= r)
            k = 1;
        else
            k = 0;

        d_k = k - last_k;

        if (d_k == 1) {
            b = path.at(i);
            if_b_asigned = true;
            b_idx = i;
            a_idx = i - 1;
            break;
        }
    }

    if (!if_b_asigned) {
        double min = 1000000;
        for (int i = 0; i < path.size(); i++) {
            if (cur_pos.distanceTo(path.at(i)) < min) {
                min = cur_pos.distanceTo(path.at(i));
                b_idx = i;
                a_idx = i - 1;
                b = path.at(i);
            }
        }
    }

    if (a_idx == -1) {
        local_goal = path.at(b_idx);
    } else {
        a = path.at(a_idx);
        double d_ca = cur_pos.distanceTo(a);
        double d_cb = cur_pos.distanceTo(b);
        local_goal.x_ = a.x_ + (b.x_ - a.x_) * (r - d_ca) / (d_cb - d_ca);
        local_goal.y_ = a.y_ + (b.y_ - a.y_) * (r - d_ca) / (d_cb - d_ca);
        local_goal.theta_ = a.theta_;
    }

    if (is_local_goal_final_reached_) {
        // cout << "local goal set to path.back()" << endl;
        local_goal = path.back();
    }

    if (cur_pos.distanceTo(path.back()) < r + 0.01)
        local_goal = path.back();

    if (local_goal.distanceTo(path.back()) < 0.005) {
        local_goal = path.back();
        is_local_goal_final_reached_ = true;
    }

    // for rviz visualization
    // geometry_msgs::PoseStamped pos_msg;
    // pos_msg.header.frame_id = map_frame_;
    // pos_msg.header.stamp = ros::Time::now();
    // pos_msg.pose.position.x = local_goal.x_;
    // pos_msg.pose.position.y = local_goal.y_;

    // tf2::Quaternion q;
    // q.setRPY(0, 0, local_goal.theta_);
    // pos_msg.pose.orientation.x = q.x();
    // pos_msg.pose.orientation.y = q.y();
    // pos_msg.pose.orientation.z = q.z();
    // pos_msg.pose.orientation.w = q.w();
    // local_goal_pub_.publish(pos_msg);

    return local_goal;
}

std::vector<RobotState> PathTracker::Orientation_Filter(std::vector<RobotState> origin_path) {
    std::vector<RobotState> path;
    double init_theta = cur_pose_.theta_;
    double goal_theta = goal_pose_.theta_;
    double theta_err = 0;
    double d_theta = 0;
    Eigen::Vector3d init;
    Eigen::Vector3d goal;
    // calculate rotate direction
    init << cos(init_theta), sin(init_theta), 0;
    goal << cos(goal_theta), sin(goal_theta), 0;

    if (init.cross(goal)(2) >= 0)
        rotate_direction_ = 1;
    else
        rotate_direction_ = -1;

    // theta_err = acos(init(0)*goal(0)+init(1)*goal(1));
    theta_err = fabs(Angle_Limit_Checking(goal_theta - init_theta));
    d_theta = rotate_direction_ * theta_err / (origin_path.size() - 1);

    RobotState point(origin_path.at(0).x_, origin_path.at(0).y_, init_theta);
    path.push_back(point);

    for (int i = 0; i < origin_path.size(); i++) {
        if (i != 0) {
            double theta;
            theta = Angle_Limit_Checking(path.at(i - 1).theta_ + d_theta);
            // cout << "theta = " << theta << endl;
            RobotState point(origin_path.at(i).x_, origin_path.at(i).y_, theta);
            path.push_back(point);
        }
    }

    // Rviz visualize processed path
    geometry_msgs::PoseArray arr_msg;
    arr_msg.header.frame_id = map_frame_;
    arr_msg.header.stamp = ros::Time::now();
    std::vector<geometry_msgs::Pose> poses;

    for (int i = 0; i < path.size(); i++) {
        geometry_msgs::Pose pose;
        pose.position.x = path.at(i).x_;
        pose.position.y = path.at(i).y_;
        tf2::Quaternion q;
        q.setRPY(0, 0, path.at(i).theta_);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        poses.push_back(pose);
    }
    arr_msg.poses = poses;
    // pose_array_pub_.publish(arr_msg);

    return path;
}

double PathTracker::Angle_Limit_Checking(double theta) {
    return fmod(theta + 2 * M_PI, 2 * M_PI);
}

// PathTracker for omni drive robot
void PathTracker::Omni_Controller(RobotState local_goal, RobotState cur_pos) {
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;

    int rotate_direction = 0;
    Eigen::Vector3d goal_vec(goal_pose_.x_, goal_pose_.y_, goal_pose_.theta_);
    Eigen::Vector3d cur_vec(cur_pos.x_, cur_pos.y_, cur_pos.theta_);

    if (cur_vec.cross(goal_vec)(2) >= 0)
        rotate_direction = 1;
    else
        rotate_direction = -1;

    // transform local_goal to base_footprint frame
    Eigen::Vector2d goal_base_vec;
    Eigen::Vector2d local_goal_bf;
    Eigen::Matrix2d rot;
    goal_base_vec << (local_goal.x_ - cur_pos.x_), (local_goal.y_ - cur_pos.y_);
    rot << cos(-cur_pos.theta_), -sin(-cur_pos.theta_), sin(-cur_pos.theta_), cos(-cur_pos.theta_);
    local_goal_bf = rot * goal_base_vec;

    t_now_ = ros::Time::now();

    dt_ = (t_now_ - t_bef_).toSec();
    // ROS_INFO("dt: %f", dt_);

    if (is_XY_Reached(cur_pose_, goal_pose_)) {
        velocity_state_.x_ = 0;
        velocity_state_.y_ = 0;
    } else {
        linear_velocity = Velocity_Profile(VELOCITY::LINEAR, cur_pose_, goal_pose_, velocity_state_, linear_acceleration_);
        double direction = atan2(local_goal_bf(1), local_goal_bf(0));
        velocity_state_.x_ = linear_velocity * cos(direction);
        velocity_state_.y_ = linear_velocity * sin(direction);
    }

    if (is_Theta_Reached(cur_pose_, goal_pose_)) {
        velocity_state_.theta_ = 0;
    } else {
        angular_velocity = Velocity_Profile(VELOCITY::ANGULAR, cur_pose_, goal_pose_, velocity_state_, rotate_direction_ * angular_acceleration_);
        velocity_state_.theta_ = angular_velocity;
    }
    Velocity_Publish();

    t_bef_ = t_now_;
}

double PathTracker::Velocity_Profile(VELOCITY vel_type, RobotState cur_pos, RobotState goal_pos, RobotState vel_state_, double acceleration) {
    double output_vel = 0;
    if (working_mode_ == MODE::TRACKING) {
        if (vel_type == VELOCITY::LINEAR) {
            RobotState _(0, 0, 0);
            double last_vel = vel_state_.distanceTo(_);
            // acceleration
            if (linear_acceleration_profile_ == "linear") {
                double d_vel = acceleration * dt_;
                output_vel = last_vel + d_vel;
            } else if (linear_acceleration_profile_ == "smooth_step") {
            }

            double xy_err = cur_pose_.distanceTo(goal_pose_);

            // deceleration
            if (xy_err < linear_brake_distance_) {
                if (linear_deceleration_profile_ == "linear") {
                    double acc = pow(linear_max_vel_, 2) / 2 / linear_brake_distance_;
                    output_vel = sqrt(2 * acc * xy_err);
                    if (output_vel < 0.25) {
                        double output_vel_ = xy_err * linear_kp_;
                        if (output_vel_ < output_vel) {
                            output_vel = output_vel_;
                        }
                    }
                } else if (linear_deceleration_profile_ == "p_control") {
                    output_vel = cur_pos.distanceTo(goal_pos) * linear_kp_;
                } else if (linear_deceleration_profile_ == "smooth_step") {
                }
            }

            // Saturation
            if (output_vel > linear_max_vel_)
                output_vel = linear_max_vel_;
        }

        if (vel_type == VELOCITY::ANGULAR) {
            double d_vel = acceleration * dt_;
            output_vel = vel_state_.theta_ + d_vel;
            double theta_err = (Angle_Limit_Checking(goal_pos.theta_ - cur_pos.theta_));

            if (fabs(theta_err) < angular_brake_distance_) {
                output_vel = theta_err * angular_kp_;
            }

            // Saturation
            if (output_vel > angular_max_vel_)
                output_vel = angular_max_vel_;
            if (output_vel < -angular_max_vel_)
                output_vel = -angular_max_vel_;
        }
    } else if (working_mode_ == MODE::TRANSITION) {
        if (vel_type == VELOCITY::LINEAR) {
            double d_vel = linear_transition_acc_ * dt_;
            RobotState _(0, 0, 0);
            double last_vel = vel_state_.distanceTo(_);
            output_vel = last_vel - d_vel;
            if (output_vel < linear_transition_vel_)
                output_vel = linear_transition_vel_;
        }

        if (vel_type == VELOCITY::ANGULAR) {
            double d_vel = angular_transition_acc_ * dt_;
            if (output_vel > 0) {
                output_vel = vel_state_.theta_ - d_vel;
                if (output_vel < angular_transition_vel_)
                    output_vel = angular_transition_vel_;
            } else {
                output_vel = vel_state_.theta_ + d_vel;
                if (output_vel > angular_transition_vel_)
                    output_vel = angular_transition_vel_;
            }
        }
    }

    return output_vel;
}

bool PathTracker::is_XY_Reached(RobotState cur_pos, RobotState goal_pos) {
    if (cur_pos.distanceTo(goal_pos) < xy_tolerance_) {
        return true;
    } else {
        return false;
    }
}

bool PathTracker::is_Theta_Reached(RobotState cur_pos, RobotState goal_pos) {
    double theta_err = 0;
    Eigen::Vector2d cur_vec;
    Eigen::Vector2d goal_vec;
    cur_vec << cos(cur_pos.theta_), sin(cur_pos.theta_);
    goal_vec << cos(goal_pos.theta_), sin(goal_pos.theta_);
    theta_err = cur_vec.dot(goal_vec);

    theta_err = fabs(Angle_Limit_Checking(goal_pos.theta_ - cur_pos.theta_));
    if (fabs(theta_err) < theta_tolerance_) {
        return true;
    } else
        return false;
}

void PathTracker::Velocity_Publish() {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = velocity_state_.x_;
    vel_msg.linear.y = velocity_state_.y_;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = velocity_state_.theta_;
    vel_pub_.publish(vel_msg);

    std_msgs::UInt8MultiArray msg;
    static int publish_state_pre = 0;
    int publish_state = 0;
    if(vel_msg.angular.z >= 0.1){
        msg.data.push_back(7);
        msg.data.push_back(3);
        publish_state = 1;
    }else if(vel_msg.angular.z <= -0.1){
        msg.data.push_back(7);
        msg.data.push_back(4);
        publish_state = 2;
    }else{
        msg.data.push_back(7);
        msg.data.push_back(9);
        publish_state = 3;  
    }
    if(publish_state_pre != publish_state){
        pub_mechanism_mission_.publish(msg);
        publish_state_pre = publish_state;
    }
    

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "PathTracker");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");
    PathTracker PathTracker_inst(nh, nh_local);

    ros::spin();
}