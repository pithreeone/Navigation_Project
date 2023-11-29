#include <ros/ros.h>
#include <obstacle_detector/Obstacles.h>
#include <tracking_system/obstacle.h>

std::vector<Obstacle> obstacles;

void targetCallback(const obstacle_detector::Obstacles::ConstPtr& msg)
{

}

int main(int argc, char** argv){
    ros::init(argc, argv, "ekf_tracker");

    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    ros::Subscriber sub_obstacles = nh.subscribe("raw_obstacles", 1000, targetCallback);


    return 0;
}