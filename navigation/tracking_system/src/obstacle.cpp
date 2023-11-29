#include <ros/ros.h>
#include <tracking_system/obstacle.h>

Obstacle::Obstacle()
{
}

Obstacle::Obstacle(double x, double y, double dt, int id)
:x_(x), y_(y), dt_(dt), id_(id)
{

}

void Obstacle::setPose(double x, double y){
    x_ = x;
    y_ = y;
}

double Obstacle::distanceBetween(double x, double y)
{
    return sqrt(pow((x - x_), 2) + pow((y - y_), 2));
}

void Obstacle::printData()
{
    ROS_INFO("EKF_Tracker: id[%d]->x:[%f], y:[%f]", id_, x_, y_);
}

bool Obstacles_handler::check_if_old_obstacle(double x, double y)
{
    bool if_old_obstacle = false;
    for(auto iter=obstacles_.begin(); iter!=obstacles_.end(); iter++){
        double d = obstacles_.distanceBetween(x, y);
        if(d <= dist_thres_){
            return true;
        }
    }
    return false;
}