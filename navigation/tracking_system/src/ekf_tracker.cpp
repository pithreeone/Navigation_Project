#include <ros/ros.h>
#include <tracking_system/obstacle.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "ekf_tracker");

    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    Obstacles_handler obstacles_handler(nh, nh_local);

    ros::spin();
    return 0;
}

