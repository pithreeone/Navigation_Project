#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#define PI 3.14159265358

double ang_ele2car = 0;    // angle between robot x-axis and elevator

// parameters
std::string base_frame;
std::string laser_frame;
double distance_thres;
double n_beams_over_thres;

// Define class number
// cc: 0, co: 1, oc: 2, oo: 3, Unknown: -1
int elevator_status = -1;
double left_mean = 0;
double right_mean = 0;

bool if_elevator_open = false;


double getTransform(){
    static tf::TransformListener listener;
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

int classifier(double left_mean, double right_mean){
    int status = -1;
    
    double ideal_closed = 1.86;
    double ideal_open = 2.4;
    if(left_mean < 0.5 || right_mean < 0.5){
        ROS_INFO("Elevator Classifier: Lidar may be blocked or not turn to right direction");
        return -1;
    }
    if(left_mean < distance_thres && right_mean < distance_thres){
        return 0;
    }else if(left_mean > distance_thres && right_mean < distance_thres){
        return 2;
    }else if(left_mean < distance_thres && right_mean > distance_thres){
        return 1;
    }else if(left_mean > distance_thres && right_mean > distance_thres){
        return 3;
    }else{
        return -1;
    }
}

void elevator_which_door_open(const sensor_msgs::LaserScan data){
    double ang_car2laser = getTransform() * 180 / PI;
    // ROS_INFO("ang2laser:%f", ang_car2laser);
    double total_ang_delta = ang_ele2car + ang_car2laser;

    double ang_check_start = 10;
    double ang_check = 35;
    double ranges_max = 5;
    double ranges_min = 0.5;
    double angle_min = data.angle_min * 180 / PI;
    std::vector<float> ranges = data.ranges;
    int n_ranges = ranges.size();
    double ang_middle = 360 - total_ang_delta;
    int n_start = ang_check_start * (n_ranges / 360.0);
    int n_calculate = ang_check * (n_ranges / 360.0);
    double ang_res = 360.0 / n_ranges;
    // ROS_INFO("ang_res:%f", ang_res);

    left_mean = right_mean = 0;
    // calculate the left mean
    int skip = 0;
    for(int i=n_start; i<n_calculate; i++){
        double temp_angle = angle_min + ang_middle + i * ang_res;
        int id = int(temp_angle / ang_res);
        // ROS_INFO("id", id);
        if(id >= n_ranges){
            id -= n_ranges;
        }else if(id < 0){
            id += n_ranges;
        }
        if(ranges[id]>ranges_max || ranges[id]<ranges_min){
            skip++;
            continue;
        }
        left_mean += ranges[id];
    }
    left_mean /= (n_calculate-n_start-skip);
    // calculate the right mean
    skip = 0;
    for(int i=n_start; i<n_calculate; i++){
        double temp_angle = angle_min + ang_middle - i * ang_res;
        int id = int(temp_angle / ang_res);
        if(id >= n_ranges){
            id -= n_ranges;
        }else if(id < 0){
            id += n_ranges;
        }
        if(ranges[id]>ranges_max || ranges[id]<ranges_min){
            skip++;
            continue;
        }
        right_mean += ranges[id];
    }
    right_mean /= (n_calculate-n_start-skip);

    ROS_INFO_THROTTLE(2, "left_mean: %f, right_mean: %f", left_mean, right_mean);
}

// check whether the door is open (inside elevator)
void elevator_if_door_open(const sensor_msgs::LaserScan data){
    int n_ranges = data.ranges.size();
    int n_beams_over = 0;

    for(int i=0; i<n_ranges; i++){
        if(data.ranges[i] > distance_thres && data.ranges[i] < data.range_max){
            n_beams_over++;
        }
    }
    if(n_beams_over > n_beams_over_thres){
        if_elevator_open = true;
    }else{
        if_elevator_open = false;
    }
    ROS_INFO_THROTTLE(2, "n_beams_over: %d", n_beams_over);

}

void laserScanCB(const sensor_msgs::LaserScan& data){
    elevator_which_door_open(data);
    elevator_if_door_open(data);
}




int main(int argc, char** argv){
    ros::init(argc, argv, "floor_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");

    nh_local.param<std::string>("base_frame", base_frame, "base");
    nh_local.param<std::string>("laser_frame", laser_frame, "laser");
    nh_local.param<double>("distance_threshold", distance_thres, 2.01);
    nh_local.param<double>("n_beams_over_thres", n_beams_over_thres, 30);

    
    ros::Subscriber scan_sub = nh.subscribe("scan_filtered", 10, laserScanCB);
    ros::Publisher elevator_pub = nh.advertise<std_msgs::Int8>("elevator_status", 100);
    ros::Publisher elevator_open_pub = nh.advertise<std_msgs::Int8>("elevator_open_status", 100);

    ros::Rate r(50);

    while(nh.ok()){
        // classify which door open
        std_msgs::Int8 msg;
        msg.data = classifier(left_mean, right_mean);
        elevator_pub.publish(msg);

        // whether door open
        msg.data = if_elevator_open;
        elevator_open_pub.publish(msg);

        r.sleep();
        ros::spinOnce();
    }
}