#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/UInt8MultiArray.h"

#define PI 3.1415926

int mission_hello = 0; // 0: stop, 1: start, 2: finish

double target_x;
double target_y;
double target_yaw;
double target_direction;  // 0: 0deg, 1: 90deg, 2: 180deg, 3: 270deg
double maximum_velocity = 0.3;
double maximum_angular_velocity = 0.7;
double kp;
double kd;
double low_pass_filter_gain;

void targetCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    target_x = msg->position.x;
    target_y = msg->position.y;

    tf2::Quaternion q;
    tf2::fromMsg(msg->orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    target_yaw = yaw;
    // ROS_INFO("I heard: [%s]", msg->data.c_str());

    // ROS_INFO("atan2: %f", atan2(target_y, target_x));

    // target_direction has the value -2 ~ 2
    target_direction = atan2(target_y, target_x) / (PI / 2);
    // ROS_INFO("dir is: %f", dir);

    ROS_INFO("Target Direction: %f", target_direction);
}

void targetStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    target_x = msg->pose.position.x;
    target_y = msg->pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    target_yaw = yaw;
    // ROS_INFO("I heard: [%s]", msg->data.c_str());

    // ROS_INFO("atan2: %f", atan2(target_y, target_x));

    // target_direction has the value -2 ~ 2
    target_direction = atan2(target_y, target_x) / (PI / 2);
    // ROS_INFO("dir is: %f", dir);

    ROS_INFO("Target Direction: %f", target_direction);
}

void trackingController(double& vx, double& vy, double& w){
    double dist = std::sqrt(target_x * target_x + target_y * target_y);
    // ROS_INFO("target_x: %f, target_y: %f", target_x, target_y);


    vx = maximum_velocity * target_x / sqrt(target_x * target_x + target_y * target_y);
    vy = maximum_velocity * target_y / sqrt(target_x * target_x + target_y * target_y);

    double stop_distance = 0.9;
    if(dist <= stop_distance){
        vx = 0;
        vy = 0;
        
        if(mission_hello == 0){
            mission_hello = 1;
        }
    }


    // angular_kp
    if((0 < target_direction && target_direction <= 0.5) || (1 < target_direction && target_direction <= 1.5) || (-2 < target_direction && target_direction <= -1.5) || (-1 < target_direction && target_direction <= -0.5)){
        w = kp * fmod((target_direction + 2), 1);
    }else if((0.5 < target_direction && target_direction <= 1) || (1.5 < target_direction && target_direction <= 2) || (-1.5 < target_direction && target_direction <= -1) || (-0.5 < target_direction && target_direction <= 0)){
        w = kp * fmod((target_direction - 2), 1);
    }
    
    // hard-constraint
    if(w >= maximum_angular_velocity){
        w = maximum_angular_velocity;
    }else if(w <= -maximum_angular_velocity){
        w = -maximum_angular_velocity;
    }    

    double angle_tolerance = 0.1;     // 0.1 ~= 6 deg
    if((fabs(target_direction - (-2)) *PI/2  <= angle_tolerance) || (fabs(target_direction - (-1))*PI/2 <= angle_tolerance*PI/2)
    || (fabs(target_direction - (0)) *PI/2 <= angle_tolerance*PI/2) || (fabs(target_direction - (1))*PI/2 <= angle_tolerance*PI/2)
    || (fabs(target_direction - (2)) *PI/2 <= angle_tolerance*PI/2))
    {
        w = 0;
    }
}

void publishVelocity(ros::Publisher pub_vel, ros::Publisher pub_mechanism, double vx, double vy, double w){
    static double pre_vx, pre_vy, pre_w;
    geometry_msgs::Twist vel;
    pre_vx = vel.linear.x = pre_vx + low_pass_filter_gain * (vx - pre_vx);
    pre_vy = vel.linear.y = pre_vy + low_pass_filter_gain * (vy - pre_vy);
    pre_w = vel.angular.z = pre_w + low_pass_filter_gain * (w - pre_w);
    pub_vel.publish(vel);
    // ROS_INFO("vx: %f, vy: %f", vx, vy);
    std_msgs::UInt8MultiArray msg;
    if(vel.angular.z >= 0.1){
        msg.data.push_back(7);
        msg.data.push_back(3);
    }else if(vel.angular.z <= -0.1){
        msg.data.push_back(7);
        msg.data.push_back(4);
    }else{
        msg.data.push_back(7);
        msg.data.push_back(10);  
    }

    pub_mechanism.publish(msg);

    

}

int main(int argc, char** argv){
    ros::init(argc, argv, "simple_cpp_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    ros::Subscriber sub_tracking_target = nh.subscribe("tracking_target", 1000, targetCallback);
    ros::Subscriber sub_tracking_target_stamped = nh.subscribe("destination_pose", 1000, targetStampedCallback);
    ros::Publisher pub_mechanism_mission = nh.advertise<std_msgs::UInt8MultiArray>("amr_mission", 1);
    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    std::string mode;
    nh_local.param<std::string>("start_mode", mode, "other");
    nh_local.param<double>("kp", kp, 1.0);
    nh_local.param<double>("kd", kd, 1.0);
    nh_local.param<double>("low_pass_filter_gain", low_pass_filter_gain, 1.0);


    ros::Rate r(100);
    while(ros::ok()){
        double vx, vy, w;
        vx = vy = w = 0;
        
        if(mission_hello == 1 && mode.compare("hello") == 0){
            static int flow = 0;

            switch(flow){
                case 0:{
                    if(target_x == 0 && target_y == 0){
                        break;
                    }
                    static double target_temp_x = target_x;
                    static double target_temp_y = target_y;
                    static double theta = atan2(target_temp_y, target_temp_x);

                    if(-PI/4 <= theta && theta <= PI/4){
                        static ros::Time start = ros::Time::now();
                        if((ros::Time::now() - start).toSec() <= ((PI/4) - theta)/maximum_angular_velocity){
                            w = maximum_angular_velocity;
                        }else{
                            flow = 1;
                        }
                    }else if(PI/4 <= theta && theta < 3*PI/4){
                        static ros::Time start = ros::Time::now();
                        if((ros::Time::now() - start).toSec() <= (theta - (PI/4))/maximum_angular_velocity){
                            w = -maximum_angular_velocity;
                        }else{
                            flow = 1;
                        }
                    }
                    ROS_INFO_THROTTLE(0.5, "[Tracker]: state flow:0");
                    break;
                }
                case 1:{
                    flow = 2;
                    ROS_INFO_THROTTLE(0.5, "[Tracker]: state flow:1");
                    break;
                }
                case 2:{
                    static ros::Time start = ros::Time::now();
                    if((ros::Time::now() - start).toSec() <= 5.0){
                        std_msgs::UInt8MultiArray msg;
                        msg.data.push_back(1);
                        msg.data.push_back(3);
                        pub_mechanism_mission.publish(msg);
                    }else{
                        flow = 3;
                        mission_hello = 2;
                        ROS_INFO("[Tracker]:Finish initial script, start tracking controller");
                    }
                    ROS_INFO_THROTTLE(0.5, "[Tracker]: state flow:2");
                    break;
                }
            }
        }else{
            trackingController(vx, vy, w);
        }

        // trackingController(vx, vy, w);
        publishVelocity(pub_vel, pub_mechanism_mission, vx, vy, w);
        // ROS_INFO_THROTTLE(0.2, "vx: %f,vy: %f,w: %f", vx, vy, w);


        r.sleep();
        ros::spinOnce();
    }

}