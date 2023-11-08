#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int32.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>

double ang_ele2car = 0;    // angle between robot x-axis and elevator

// parameters
std::string base_frame;
std::string laser_frame;



double getTransform(){
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try{
        if(listener.waitForTransform(base_frame, laser_frame, ros::Time(0), ros::Duration(1.0))){
            listener.lookupTransform(base_frame, laser_frame, ros::Time(0), transform);
            // ROS_INFO("transform successfully");
        }
    }
    catch(tf::TransformException &ex){
        ROS_WARN("%s", ex.what());
    }

    tf2::Quaternion q;
    q.setX(transform.getRotation().x());
    q.setY(transform.getRotation().y());
    q.setZ(transform.getRotation().z());
    q.setW(transform.getRotation().w());

    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    return yaw;
}

void laserScanCB(const sensor_msgs::LaserScan& data){
    double ang_car2laser = getTransform();
    double total_ang_delta = ang_ele2car + ang_car2laser;
    double left_mean, right_mean;
    double ang_check = 45;
    std::vector<float> ranges = data.ranges;
    int n_ranges = ranges.size();
    double ang_middle = 360 - total_ang_delta;
    int n_calculate = ang_check * (n_ranges / 360.0);
    double ang_res = 360.0 / n_ranges;
    // calculate the left mean
    for(int i=0; i<n_calculate; i++){
        double temp_angle = ang_middle + i * ang_res;
        int id = int(temp_angle / ang_res);
        if(id >= n_ranges){
            id -= n_ranges;
        }else if(id < 0){
            id += n_ranges;
        }
        left_mean += ranges[id];
    }
    left_mean /= n_calculate;
    // calculate the right mean
    for(int i=0; i<n_calculate; i++){
        double temp_angle = ang_middle - i * ang_res;
        int id = int(temp_angle / ang_res);
        if(id >= n_ranges){
            id -= n_ranges;
        }else if(id < 0){
            id += n_ranges;
        }
        right_mean += ranges[id];
    }
    right_mean /= n_calculate;

    ROS_INFO("left_mean: %f, right_mean: %f", left_mean, right_mean);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "floor_publisher");
    ros::NodeHandle nh;

    nh_local.param<std::string>("base_frame", base_frame, "base");
    nh_local.param<std::string>("laser_frame", laser_frame, "laser");

    ros::Subscriber scan_sub = nh.subscribe("scan_filtered", 10, laserScanCB);
    ros::Publisher elevator_pub = nh.advertise<std_msgs::Int32>("elevator_status", 10);

    ros::Rate r(50);

    while(nh.ok()){

        r.sleep();
        ros::spinOnce();
    }
}