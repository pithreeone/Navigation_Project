#include <ros/ros.h>
#include <tracking_system/obstacle.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Dense>

Obstacle::Obstacle()
{
}

Obstacle::Obstacle(double x, double y, double dt, int id, ros::Time stamp)
:x_(x), y_(y), dt_(dt), id_(id), stamp_(stamp)
{
    vx_ = vy_ = 0;
    cov_mat << 0.1, 0, 0, 0,
               0, 0.1, 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1;

    A << 1, 0, dt_, 0,
         0, 1, 0, dt_,
         0, 0, 1, 0,
         0, 0, 0, 1;

    C << 1, 0, 0, 0,
         0, 1, 0, 0,    

    Q << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;

    R << 0.1, 0,
         0, 0.1; 

}

void Obstacle::setPose(double x, double y){
    x_ = x;
    y_ = y;
}

double Obstacle::distanceBetween(double x, double y)
{
    return sqrt(pow((x - x_), 2) + pow((y - y_), 2));
}

void Obstacle::KF_predict()
{
    state << x_, y_, vx_, vy_;
    state = A * state;
    cov_mat = A * cov_mat * A.transpose() + Q;
    
}

void Obstacle::KF_update(double x, double y)
{
    Eigen::Matrix<double, 2, 1> measurement;
    measurement << x, y;
    Eigen::Matrix<double, 4, 2> K;
    K = cov_mat * C.transpose() * (C * cov_mat * C.transpose() + R).inverse();
    state = state + K * (measurement - C * state);
    Eigen::Matrix<double, 4, 4> identityMatrix;
    identityMatrix << 1, 0, 0, 0,
                      0, 1, 0, 0, 
                      0, 0, 1, 0,
                      0, 0, 0, 1;
    cov_mat = (identityMatrix - K * C) * cov_mat;
}

void Obstacle::printData()
{
    ROS_INFO("EKF_Tracker: id[%d]->x:[%f], y:[%f]", id_, x_, y_);
}

void Obstacle::printState()
{
    std::cout << state << std::endl;
}

Obstacles_handler::Obstacles_handler(ros::NodeHandle& nh, ros::NodeHandle& nh_local)
{
    nh_ = nh;
    nh_local_ = nh_local;
    initialize();
}

void Obstacles_handler::initializeParam()
{
    nh_local_.param<double>("distance_threshold", dist_thres_, 0.15);
    // ROS_INFO("distance_threshold:%f", dist_thres_);
}

void Obstacles_handler::initialize()
{
    initializeParam();
    sub_obstacles_ = nh_.subscribe("raw_obstacles", 1000, &Obstacles_handler::targetCallback, this);
    sub_target_pose_ = nh_.subscribe("set_target", 1000, &Obstacles_handler::setTargetCallback, this);
    pub_target_ = nh_.advertise<geometry_msgs::PoseStamped>("tracking_target_2", 10);
    pub_target_pose_ = nh_.advertise<geometry_msgs::Pose>("tracking_target", 10);
    pub_obstacles_filtered_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("obstacles_filtered", 50);
    pub_obstacles_array_filtered_ = nh_.advertise<geometry_msgs::PoseArray>("obstacles_array_filtered", 50);

}

void Obstacles_handler::targetCallback(const obstacle_detector::Obstacles::ConstPtr& msg)
{   
    static bool if_initialize = false;
    // predictObstacle();

    double x, y;
    double dist = 100000;
    for(auto i=msg->circles.begin(); i!=msg->circles.end(); i++){

        geometry_msgs::PointStamped transformed_point;
        transformed_point = obstacle_frame_transform(msg->header.frame_id, "odom", i->center.x, i->center.y);
        int id = checkIfOldObstacle(transformed_point.point.x , transformed_point.point.y);
        if(id == -1){
            pushObstacle(transformed_point.point.x, transformed_point.point.y);
        }else{
            updateObstacle(id, transformed_point.point.x, transformed_point.point.y);
        }
        if(sqrt(pow(i->center.x,2) + pow(i->center.y,2)) <= dist){
            x = i->center.x;
            y = i->center.y;
            dist = sqrt(pow(i->center.x,2) + pow(i->center.y,2));
        }
    }
    
    // initialize target obstacle. Find the nearest obstacle.
    if(if_initialize == false){
        target_obstacle_.setPose(x, y);
        ROS_INFO("Initialize target pose: (%f, %f)", x, y);
        if_initialize = true;
    }
    findNearestObstacleAndPublish();
    clearOldObstacle();
    visualizeObstacleFiltered();
    // static ros::Time t_last;
    // if((ros::Time::now() - t_last).toSec() > 0.1){
    //     visualizeObstacleFiltered();
    //     t_last = ros::Time::now();
    //     // ROS_INFO("visualize");
    // }
    
}

void Obstacles_handler::setTargetCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    target_obstacle_.setPose(x, y);

}

geometry_msgs::PointStamped Obstacles_handler::obstacle_frame_transform(std::string original_frame, std::string target_frame, double x, double y)
{
    geometry_msgs::PointStamped original_point;
    original_point.header.frame_id = original_frame;
    original_point.point.x = x;
    original_point.point.y = y;
    original_point.point.z = 0;
    // Transform the coordinate to the target frame
    try {
        tf_listener_.waitForTransform(target_frame, original_point.header.frame_id,
                                ros::Time(0), ros::Duration(1.0));
        tf_listener_.transformPoint(target_frame, original_point, original_point);
        // ROS_INFO("Original Point: (%f, %f, %f) in %s frame",
        //         original_point.point.x, original_point.point.y, original_point.point.z,
        //         original_point.header.frame_id.c_str());
        // ROS_INFO("Transformed Point: (%f, %f, %f) in target_frame",
        //         original_point.point.x, original_point.point.y, original_point.point.z);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("Transform failed: %s", ex.what());
    }
    return original_point;
}

int Obstacles_handler::checkIfOldObstacle(double x, double y)
{
    bool if_old_obstacle = false;
    double d_min = 100000;
    std::vector<Obstacle>::iterator iter_min;
    for(auto iter=obstacles_.begin(); iter!=obstacles_.end(); iter++){
        double d = iter->distanceBetween(x, y);
        if(d <= d_min){
            d_min = d;
            iter_min = iter;
        }
    }
    
    if(d_min <= dist_thres_){
        return std::distance(obstacles_.begin(), iter_min);
    }else{
        return -1;
    }
}

void Obstacles_handler::pushObstacle(double x, double y)
{
    Obstacle obs(x, y, dt_, obstacles_.size(), ros::Time::now());
    obstacles_.push_back(obs);
}

void Obstacles_handler::predictObstacle()
{
    for(auto iter=obstacles_.begin(); iter!=obstacles_.end(); iter++){
        iter->KF_predict();
        // iter->printState();
    }
}

void Obstacles_handler::updateObstacle(int id, double x, double y)
{
    // euclidean-method
    obstacles_[id].setPose(x, y);
    obstacles_[id].setStamped(ros::Time::now());

    // Kalman-Filter-method
    // obstacles_[id].KF_update(x, y);
}

void Obstacles_handler::findNearestObstacleAndPublish()
{   
    double dist = 10000000;
    std::vector<Obstacle>::iterator iter_min;
    double x, y;
    for(auto i=obstacles_.begin(); i!=obstacles_.end(); i++){
        
        target_obstacle_.getXY(x, y);
        if(i->distanceBetween(x, y) <= dist){
            dist = i->distanceBetween(x, y);
            iter_min = i;
        }
    }
    iter_min->getXY(x, y);
    double x_past, y_past;
    target_obstacle_.getXY(x_past, y_past);

    double max_distance_jump = 0.35;
    ROS_INFO("x: %f, y: %f, x_past: %f, y_past: %f", x, y, x_past, y_past);
    if(sqrt(pow((x - x_past), 2) + pow((y - y_past), 2)) < max_distance_jump){
        target_obstacle_.setPose(x, y);
    }
    ROS_INFO("diff: %f", sqrt(pow((x - x_past), 2) + pow((y - y_past), 2)));
    

    target_obstacle_.getXY(x, y);
    // transform to base_link frame
    geometry_msgs::PointStamped original_point;
    original_point.header.frame_id = "odom";
    ros::Duration delay(0.02);
    original_point.header.stamp = ros::Time::now() - delay;
    original_point.point.x = x;
    original_point.point.y = y;
    original_point.point.z = 0;
    std::string target_frame = "base_link";
    // Transform the coordinate to the target frame
    try {
        // ROS_INFO("Original Point: (%f, %f, %f) in %s frame",
        //         original_point.point.x, original_point.point.y, original_point.point.z,
        //         original_point.header.frame_id.c_str());

        tf_listener_.waitForTransform(target_frame, original_point.header.frame_id,
                                ros::Time(0), ros::Duration(2.0));
        tf_listener_.transformPoint(target_frame, original_point, original_point);

        // ROS_INFO("Transformed Point: (%f, %f, %f) in target_frame",
        //         original_point.point.x, original_point.point.y, original_point.point.z);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("Transform failed: %s", ex.what());
    }

    // publish Topic for visualization. <geometry_msgs::PoseStamped>
    geometry_msgs::PoseStamped target_pose_poseStamped;
    target_pose_poseStamped.header.frame_id = original_point.header.frame_id;
    target_pose_poseStamped.header.stamp = ros::Time::now();
    target_pose_poseStamped.pose.position.x = original_point.point.x;
    target_pose_poseStamped.pose.position.y = original_point.point.y;
    pub_target_.publish(target_pose_poseStamped);

    // publish Topic for tracker.cpp <geometry_msgs::Pose>
    geometry_msgs::Pose target_pose_pose;
    target_pose_pose.position.x = original_point.point.x;
    target_pose_pose.position.y = original_point.point.y;
    pub_target_pose_.publish(target_pose_pose);
}

void Obstacles_handler::clearOldObstacle()
{
    for(auto iter=obstacles_.begin(); iter!=obstacles_.end(); ){
        if((ros::Time::now() - iter->getStamped()).toSec() > 1.0){
            obstacles_.erase(iter);
            // ROS_INFO("Erase");
        }else{
            iter++;
        }
    } 
}

void Obstacles_handler::visualizeObstacleFiltered()
{
    geometry_msgs::PoseArray pose_array_msg;
    pose_array_msg.header.frame_id = "odom";
    pose_array_msg.header.stamp = ros::Time::now();
    for(auto iter=obstacles_.begin(); iter!=obstacles_.end(); iter++){
        geometry_msgs::PoseWithCovarianceStamped temp;
        temp.header.frame_id = "odom";
        temp.header.stamp = ros::Time::now();
        double x, y;
        iter->getXY(x, y);
        temp.pose.pose.position.x = x;
        temp.pose.pose.position.y = y;
        temp.pose.pose.orientation.w = 1;
        // pub_obstacles_filtered_.publish(temp);
        
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.orientation.w = 1;
        pose_array_msg.poses.push_back(pose);
    }
    pub_obstacles_array_filtered_.publish(pose_array_msg);
}

