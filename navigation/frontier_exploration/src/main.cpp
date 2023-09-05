#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Char.h"
#include "frontier_exploration/main.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <mutex>

void Frontier_Exploration::initialize()
{
    sub_map_ = nh_.subscribe("map", 100, &Frontier_Exploration::mapCallback, this);
    sub_odom_feedback_ = nh_.subscribe("finishornot", 100, &Frontier_Exploration::odomFeedbackCallback, this);
    pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("nav_goal", 1);
    pub_frontier_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("frontier_map", 1);
    pub_frontier_groups_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("frontier_groups_map", 1);
    pub_frontier_center_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("frontier_center_map", 1);

    initializeParam();
    if_set_map_ = false;
    timer_ = nh_.createTimer(ros::Duration(double(1/process_frequency_)), &Frontier_Exploration::processCallback, this);
    state_ = Process_State::Stop;

}

void Frontier_Exploration::initializeParam()
{

    nh_local_.param<std::string>("map_frame", map_frame_, "map");
    nh_local_.param<std::string>("base_frame", base_frame_, "base");
    nh_local_.param<int>("thresh", thresh_, 50);
    nh_local_.param<int>("clear_frontiers", clear_frontiers_, 10);
    nh_local_.param<double>("process_frequency", process_frequency_, 1);
}

void Frontier_Exploration::getRobotPose()
{
    try{
        if(listener_.waitForTransform(map_frame_, base_frame_, ros::Time(0), ros::Duration(1.0))){
            listener_.lookupTransform(map_frame_, base_frame_, ros::Time(0), transform_);
        }
    }
    catch(tf::TransformException &ex){
        ROS_WARN("%s", ex.what());
    }
    cur_pose_.x_ = transform_.getOrigin().x();
    cur_pose_.y_ = transform_.getOrigin().y();
    tf::Quaternion q;
    q.setX(transform_.getRotation().x());
    q.setY(transform_.getRotation().y());
    q.setZ(transform_.getRotation().z());
    q.setW(transform_.getRotation().w());

    tf::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    cur_pose_.theta_ = yaw;
}

void Frontier_Exploration::copyGridMap(nav_msgs::OccupancyGrid map, nav_msgs::OccupancyGrid& map_copy)
{
    map_copy.header.stamp = map.header.stamp;
    map_copy.header.frame_id = map.header.frame_id;
    map_copy.info.resolution = map.info.resolution;
    map_copy.info.width = map.info.width;
    map_copy.info.height = map.info.height;
    map_copy.info.origin.position.x = map.info.origin.position.x;
    map_copy.info.origin.position.y = map.info.origin.position.y;
    map_copy.info.origin.position.z = map.info.origin.position.z;
    map_copy.info.origin.orientation.x = map.info.origin.orientation.x;
    map_copy.info.origin.orientation.y = map.info.origin.orientation.y;
    map_copy.info.origin.orientation.z = map.info.origin.orientation.z;
    map_copy.info.origin.orientation.w = map.info.origin.orientation.w;
    if(map_copy.data.size() != map.data.size()){
        map_copy.data.resize(map.data.size());
    }
    for(int i=0; i<map.info.width; i++){
        for(int j=0; j<map.info.height; j++){
            map_copy.data[j*map.info.width+i] = map.data[j*map.info.width+i];
        }
    }
}

void Frontier_Exploration::printProcessState(){
    switch(state_){
        case Process_State::Stop:
            ROS_INFO("Frontier_Exploration: Process state -> Stop");
            break;
        case Process_State::Explore:
            ROS_INFO("Frontier_Exploration: Process state -> Explore");
            break;
        case Process_State::Move:
            ROS_INFO("Frontier_Exploration: Process state -> Move");
            break;
        case Process_State::Arrive:
            ROS_INFO("Frontier_Exploration: Process state -> Arrive");
            break;
        case Process_State::Finish_Explore:
            ROS_INFO("Frontier_Exploration: Process state -> Finish_Explore");
            break;
    }
}

void Frontier_Exploration::WFD()
{
    if(if_set_map_ == false){
        ROS_WARN("Frontier_exploration: Can not subscribe to map-topic");
        return;
    }
    // initialize
    if(!frontier_points_.empty())
        frontier_points_.clear();
    n_frontier_group_ = 0;
    copyGridMap(map_buffer_, map_);
    origin_map_.setMap(map_);
    origin_map_.setThresh(thresh_); 
    // ROS_INFO("width: %d, height: %d, resolution: %f", origin_map_.getWidth(), origin_map_.getHeight(), origin_map_.getResolution());

    // line-1
    std::queue <Pose> qm;
    // line-2
    qm.push(cur_pose_);
    // // line-3
    int cur_cell_x = 0, cur_cell_y = 0;
    origin_map_.worldToMap(cur_pose_, cur_cell_x, cur_cell_y);
    // std::cout << "x" << cur_cell_x << " y" << cur_cell_y << std::endl;
    origin_map_.setIndicatorData(cur_cell_x, cur_cell_y, Point_Indicator::Map_Open_List);

    // line-4
    while(!qm.empty()){
        // line-5
        Pose p = qm.front();
        qm.pop();
        // line-6
        int px, py;
        origin_map_.worldToMap(p, px, py);
        // std::cout << "x:" << px << " y:" << py << std::endl;
        if(origin_map_.getIndicatorData(px, py) == Point_Indicator::Map_Close_List)
            continue;
        // line-8
        if(origin_map_.ifFrontierPoint(px, py)){
            // line-9
            std::queue <Pose> qf;
            // line-10
            std::queue <Pose> new_frontier;
            // line-11
            qf.push(p);
            // line-12
            origin_map_.setIndicatorData(px, py, Point_Indicator::Frontier_Open_List);

            // line-13
            while(!qf.empty()){
                // line-14
                Pose q = qf.front();
                qf.pop();
                // line-15
                int qx, qy;
                origin_map_.worldToMap(q, qx, qy);
                if((origin_map_.getIndicatorData(qx, qy) == Point_Indicator::Map_Close_List)
                    || (origin_map_.getIndicatorData(qx, qy) == Point_Indicator::Frontier_Close_List))
                    continue;
                // line-17
                if(origin_map_.ifFrontierPoint(qx, qy)){
                    // line-18
                    new_frontier.push(q);
                    // line-19
                    for(int i= int(Direction::Front); i<=int(Direction::Right); i++){
                        int wx = qx, wy = qy;
                        if(i==int(Direction::Front)){
                            if(qy>=origin_map_.getHeight()-1)
                                continue;
                            wy++;
                        }else if(i==int(Direction::Left)){
                            if(qx<=0)
                                continue;
                            wx--;
                        }else if(i==int(Direction::Back)){
                            if(qy<=0)
                                continue;
                            wy--;
                        }else if(i==int(Direction::Right)){
                            if(qx>=origin_map_.getWidth()-1)
                                continue;
                            wx++;
                        }
                        // line-20
                        if(origin_map_.getIndicatorData(wx, wy)!=Point_Indicator::Frontier_Open_List
                        && origin_map_.getIndicatorData(wx, wy)!=Point_Indicator::Frontier_Close_List
                        && origin_map_.getIndicatorData(wx, wy)!=Point_Indicator::Map_Close_List){
                            // line-21
                            Pose w;
                            origin_map_.mapToWorld(wx, wy, w);
                            qf.push(w);
                            // line-22
                            origin_map_.setIndicatorData(wx, wy, Point_Indicator::Frontier_Open_List);
                        }
                    }
                }
                // line-23
                origin_map_.setIndicatorData(qx, qy, Point_Indicator::Frontier_Close_List);
            }
            // line-24
            if(frontier_points_.empty()){
                n_frontier_group_ = 0;
            }
            while(!new_frontier.empty()){
                Pose p_f = new_frontier.front();
                new_frontier.pop();
                Frontier frontier_p(p_f, n_frontier_group_);
                frontier_points_.push_back(frontier_p);
                // line-25
                int pfx, pfy;
                origin_map_.mapToWorld(pfx, pfy, p_f);
                origin_map_.setIndicatorData(pfx, pfy, Point_Indicator::Map_Close_List);
            }
            n_frontier_group_++;
        }
        // line-26
        // ROS_INFO("vx: %d, vy: %d", vx, vy);
        for(int i=int(Direction::Front); i<=int(Direction::Right); i++){
            int vx = px, vy = py;
            if(i==int(Direction::Front)){
                if(py>=origin_map_.getHeight()-1)
                    continue;
                vy++;
            }else if(i==int(Direction::Left)){
                if(px<=0)
                    continue;
                vx--;
            }else if(i==int(Direction::Back)){
                if(py<=0)
                    continue;
                vy--;
            }else if(i==int(Direction::Right)){
                if(px>=origin_map_.getWidth()-1)
                    continue;
                vx++;
            }        
            // line-27
            // std::cout << "vx" << vx << " vy" << vy << std::endl;
            if(origin_map_.getIndicatorData(vx, vy)!=Point_Indicator::Map_Open_List
            && origin_map_.getIndicatorData(vx, vy)!=Point_Indicator::Map_Close_List
            && origin_map_.ifHasMapOpenSpaceNeighbor(vx, vy)){
                // line-28
                Pose v;
                origin_map_.mapToWorld(vx, vy, v);
                qm.push(v);
                // line-29
                origin_map_.setIndicatorData(vx, vy, Point_Indicator::Map_Open_List);
            }
        }
        // line-30
        origin_map_.setIndicatorData(px, py, Point_Indicator::Map_Close_List);
    }
    // debug test, publish the point that has been finded
    // nav_msgs::OccupancyGrid frontier_groups_map;
    // frontier_groups_map.info = map_.info;
    // for(int i=0; i<map_.info.width * map_.info.height; i++){
    //     frontier_groups_map.data.push_back(0);
    // }
    // for(int i=0;i<origin_map_.getWidth(); i++){
    //     for(int j=0;j<origin_map_.getHeight(); j++){
    //         if(origin_map_.getIndicatorData(i, j)==Point_Indicator::Map_Close_List)
    //             frontier_groups_map.data[j*origin_map_.getWidth()+i]=100;
    //     }
    // }

    // pub_frontier_groups_map_.publish(frontier_groups_map); 
}

int Frontier_Exploration::deleteSmallFrontier()
{
    n_frontier_group_ = 0;
    std::vector<Frontier> temp;
    for(int i=0; i<frontier_points_.size(); i++){
        int n = 1;
        while(i!=frontier_points_.size()-1 && frontier_points_[i].id_ == frontier_points_[i+1].id_){
            i++;
            n++;
        }
        if(n > clear_frontiers_){
            for(int j=0; j<n; j++){
                temp.push_back(frontier_points_[i-j]);
            }
            n_frontier_group_++;
        }
    }
    frontier_points_.clear();
    for(int i=0;i<temp.size(); i++){
        frontier_points_.push_back(temp[i]);
    }
    return n_frontier_group_;
}

bool Frontier_Exploration::calculateNearestFrontier(Pose& p)
{
    if(!origin_map_.ifSetMap())
        return false;
    if(n_frontier_group_ == 0)
        return false;

    int n = 0;
    double min_dist = 10000000;
    int min_id;
    // std::cout << "n_frontier_group: " << n_frontier_group_ << std::endl;
    // std::cout << "frontier_points.size: " << frontier_points_.size() << std::endl;
    frontier_centers_.resize(n_frontier_group_);
    for(int i=0; i<n_frontier_group_; i++){
        int groups_n = 0;
        // find center of a group of frontier
        do{
            frontier_centers_[i] += frontier_points_[n++];
            groups_n++;
            if(n == frontier_points_.size())
                break;
        }while(frontier_points_[n].id_ == frontier_points_[n-1].id_);
        frontier_centers_[i] /= groups_n;
        // find minimum distance between center and current pose
        Pose diff = frontier_centers_[i] - cur_pose_;
        double d = diff.distance();
        if(d < min_dist){
            min_dist = d;
            min_id = i;
        }
    }
    p = frontier_centers_[min_id];
    publishFrontierGroupsMap();
    publishFrontierCenterMap();
    return true;
}

void Frontier_Exploration::publishFrontierMap()
{       
    copyGridMap(map_buffer_, map_);
    origin_map_.setMap(map_);
    origin_map_.setThresh(thresh_); 
    nav_msgs::OccupancyGrid frontier_map;
    frontier_map.info = map_.info;
    for(int i=0; i<map_buffer_.info.width * map_buffer_.info.height; i++){
        frontier_map.data.push_back(0);
    }

    for(int i=0; i< map_.info.width; i++){
        for(int j=0; j<map_.info.height; j++){
            if(origin_map_.ifFrontierPoint(i, j) == true)
                frontier_map.data[j * map_.info.width + i] = 100;
            else
                frontier_map.data[j * map_.info.width + i] = 0;
        }
    }
    pub_frontier_map_.publish(frontier_map);
}

void Frontier_Exploration::publishFrontierGroupsMap()
{
    if(!origin_map_.ifSetMap())
        return;
    nav_msgs::OccupancyGrid frontier_groups_map;
    frontier_groups_map.info = map_.info;
    frontier_groups_map.data.resize(origin_map_.getWidth() * origin_map_.getHeight());
    for(int i=0; i<frontier_points_.size(); i++){
        int fx, fy;
        origin_map_.worldToMap(frontier_points_[i], fx, fy);
        frontier_groups_map.data[fy * origin_map_.getWidth() + fx] = 100;
    }
    pub_frontier_groups_map_.publish(frontier_groups_map); 
}

void Frontier_Exploration::publishFrontierCenterMap()
{
    if(!origin_map_.ifSetMap())
        return;
    nav_msgs::OccupancyGrid frontier_center_map;
    frontier_center_map.info = map_.info;
    frontier_center_map.data.resize(origin_map_.getWidth() * origin_map_.getHeight());

    for(int i=0; i<n_frontier_group_; i++){
        int fx, fy;
        origin_map_.worldToMap(frontier_centers_[i], fx, fy);
        frontier_center_map.data[fy * origin_map_.getWidth() + fx] = 100;
    }
    pub_frontier_center_map_.publish(frontier_center_map); 
}

void Frontier_Exploration::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) 
{   
    copyGridMap(*msg, map_buffer_);
    ROS_INFO("Frontier_Exploration: map Updated!");
    if_set_map_ = true;
}

void Frontier_Exploration::odomFeedbackCallback(const std_msgs::Char::ConstPtr & msg)
{
    if(msg->data == 1){
        setProcessState(Process_State::Arrive);
    }
}

void Frontier_Exploration::processCallback(const ros::TimerEvent &)
{
    printProcessState();
    switch(state_){
        case Process_State::Stop:
        {
            if(!if_set_map_){
                ROS_WARN("Frontier_exploration: Can not subscribe to map-topic");
                return;
            }
            getRobotPose();
            WFD();
            int n = deleteSmallFrontier();
            if(n >= 1){
                setProcessState(Process_State::Explore);
            }else{
                setProcessState(Process_State::Finish_Explore);
            }
            break;
        }
        case Process_State::Explore:
        {
            Pose nearest_p;
            bool if_frontiers = calculateNearestFrontier(nearest_p);
            geometry_msgs::PoseStamped goal;
            uint32_t seq = 0;
            goal.header.seq = seq++;
            goal.header.stamp = ros::Time::now();
            goal.header.frame_id = map_frame_;
            goal.pose.position.x = nearest_p.x_;
            goal.pose.position.y = nearest_p.y_;
            goal.pose.position.z = 0;
            tf::Quaternion q;
            q.setRPY(0, 0, cur_pose_.theta_);
            goal.pose.orientation.x = q.x();
            goal.pose.orientation.y = q.y();
            goal.pose.orientation.z = q.z();
            goal.pose.orientation.w = q.w();
            pub_goal_.publish(goal);
            setProcessState(Process_State::Move);
            break;
        }
        case Process_State::Move:
        {
            break;
        }
        case Process_State::Arrive:
        {
            setProcessState(Process_State::Stop);
            break;
        }
        case Process_State::Finish_Explore:
        {
            break;
        }
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "frontier_exploration");
    ros::NodeHandle nh(""); 
    ros::NodeHandle nh_local("~");
    ros::Rate r(100);

    Frontier_Exploration frontier_exploration(nh, nh_local);
    frontier_exploration.initialize();
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }

}