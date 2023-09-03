#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "frontier_exploration/main.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <mutex>

void Frontier_Exploration::initialize()
{
    sub_map_ = nh_.subscribe("map", 100, &Frontier_Exploration::mapCallback, this);
    pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("nav_goal", 1);
    pub_frontier_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("frontier_map", 1);
    pub_frontier_groups_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("frontier_groups_map", 1);
    initializeParam();
    cur_pose_x_ = 0;
    cur_pose_y_ = 0;
    cur_pose_theta_ = 0;
    if_map_updated_ = false;

}

void Frontier_Exploration::initializeParam()
{

    nh_local_.param<std::string>("map_frame", map_frame_, "map");
    nh_local_.param<std::string>("base_frame", base_frame_, "base");
    nh_local_.param<int>("thresh", thresh_, 50);
    nh_local_.param<int>("clear_frontiers", clear_frontiers_, 10);
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
    cur_pose_x_ = transform_.getOrigin().x();
    cur_pose_y_ = transform_.getOrigin().y();
    tf::Quaternion q;
    q.setX(transform_.getRotation().x());
    q.setY(transform_.getRotation().y());
    q.setZ(transform_.getRotation().z());
    q.setW(transform_.getRotation().w());

    tf::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    cur_pose_theta_ = yaw;
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

void Frontier_Exploration::WFD()
{
    if(if_map_updated_ == false){
        ROS_INFO("Frontier_exploration: waiting for map updated");
        return;
    }
    if_map_updated_ = false;
    // initialize
    while(!frontier_points_.empty())
        frontier_points_.pop();
    n_frontier_group_ = 0;
    copyGridMap(map_buffer_, map_);
    origin_map_.setMap(map_);
    origin_map_.setThresh(thresh_); 
    // ROS_INFO("width: %d, height: %d, resolution: %f", origin_map_.getWidth(), origin_map_.getHeight(), origin_map_.getResolution());

    // line-1
    std::queue <Pose> qm;
    // line-2
    Pose cur_pose(cur_pose_x_, cur_pose_y_, cur_pose_theta_);
    qm.push(cur_pose);
    // // line-3
    int cur_cell_x = 0, cur_cell_y = 0;
    origin_map_.worldToMap(cur_pose, cur_cell_x, cur_cell_y);
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
                frontier_points_.push(frontier_p);
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

void Frontier_Exploration::publishFrontierGroupsMap()
{
    WFD();
    nav_msgs::OccupancyGrid frontier_groups_map;
    frontier_groups_map.info = map_.info;
    frontier_groups_map.data.resize(origin_map_.getWidth() * origin_map_.getHeight());

    int n_groups = 0;
    while(!frontier_points_.empty()){
        int n_frontier = 0;
        int id;
        std::queue <Frontier> q_frontier_buf;
        do{
            Frontier f = frontier_points_.front();
            frontier_points_.pop();
            q_frontier_buf.push(f);
            id = f.id_;
            n_frontier++;
        }while(id == frontier_points_.front().id_);

        if(n_frontier <= clear_frontiers_)
            continue;

        int f_cell_x, f_cell_y;
        while(!q_frontier_buf.empty()){
            Frontier f = q_frontier_buf.front();
            q_frontier_buf.pop();
            origin_map_.worldToMap(f, f_cell_x, f_cell_y);
            // std::cout << id << " ";
            frontier_groups_map.data[f_cell_y * origin_map_.getWidth() + f_cell_x] = 100;
        }
        n_groups++;
    }
    ROS_INFO("Frontier_exploration: There are %d groups of frontier", n_groups);
    pub_frontier_groups_map_.publish(frontier_groups_map); 
}

void Frontier_Exploration::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) 
{   
    copyGridMap(*msg, map_buffer_);
    ROS_INFO("Frontier_Exploration: map Updated!");
    if_map_updated_ = true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "frontier_exploration");
    ros::NodeHandle nh(""); 
    ros::NodeHandle nh_local("~");
    ros::Rate r(1);

    Frontier_Exploration frontier_exploration(nh, nh_local);
    frontier_exploration.initialize();
    while(ros::ok()){
        // frontier_exploration.publishFrontierMap();
        frontier_exploration.publishFrontierGroupsMap();
        ros::spinOnce();
        r.sleep();
    }

}