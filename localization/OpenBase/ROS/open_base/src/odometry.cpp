#include <string>

#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/JointState.h>

#include <kdl/frames.hpp>

#include <open_base/KinematicsForward.h>
#include <open_base/Velocity.h>

long double duration;

int i;

ros::ServiceClient kinematicsForwardMobile;

ros::ServiceClient kinematicsForwardWorld;

std::string openBaseString = "open_base";

std::string originString = "origin";

geometry_msgs::Pose2D pose;

geometry_msgs::Pose poseCheat;

geometry_msgs::Pose2D poseMobile;

geometry_msgs::Pose2D poseWorld;

ros::Publisher publisherMobile;

ros::Publisher publisherWorld;

long double r;

double rotX, rotY, rotZ;

KDL::Rotation rotation;

open_base::KinematicsForward service;

ros::Time timeCurrent;
ros::Time timePrevious;

void onEncoderMessage(const open_base::Velocity::ConstPtr& input){
    service.request.input.v_left  = input->v_left  * r;
    service.request.input.v_back  = input->v_back  * r;
    service.request.input.v_right = input->v_right * r;
    timeCurrent = ros::Time::now();
    duration = (timeCurrent - timePrevious).toSec();
    timePrevious = timeCurrent;

    kinematicsForwardMobile.call(service);
    // transform to odom frame
    double vx_odom = service.response.output.x;
    double vy_odom = service.response.output.y;
    double w_odom = service.response.output.theta;
    double delta_x = (vx_odom * cos(poseMobile.theta) - vy_odom * sin(poseMobile.theta)) * duration;
    double delta_y = (vx_odom * sin(poseMobile.theta) + vy_odom * cos(poseMobile.theta)) * duration;
    double delta_th = w_odom * duration;
    // ROS_INFO("dx:%f, dy:%f, dth:%f", delta_x, delta_y, delta_th);
    poseMobile.x     = delta_x + poseMobile.x    ;
    poseMobile.y     = delta_y + poseMobile.y    ;
    poseMobile.theta = delta_th + poseMobile.theta;
    if ((!std::isnan(poseMobile.x)) && (!std::isnan(poseMobile.y)) && (!std::isnan(poseMobile.theta))) {
        publisherMobile.publish(poseMobile);
    }
}

void onGazeboMessage(const gazebo_msgs::LinkStates::ConstPtr& input){
    for (i = 0; i < input->name.size(); i++) {
        if (((input->name[i]).find(openBaseString) == std::string::npos) || ((input->name[i]).find(originString) == std::string::npos)) {
            continue;
        }
        poseCheat = input->pose[i];
        rotation = KDL::Rotation::Quaternion(poseCheat.orientation.x, poseCheat.orientation.y, poseCheat.orientation.z, poseCheat.orientation.w);
        rotation.GetRPY(rotX, rotY, rotZ);
        poseWorld.x     = poseCheat.position.x;
        poseWorld.y     = poseCheat.position.y;
        poseWorld.theta = rotZ;
        publisherWorld.publish(poseWorld);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "odometry");
    ros::NodeHandle node;
    while (!ros::Time::waitForValid()) {}
    {
        double parameter;
        if (!node.getParam("parameter/wheel/radius", parameter)) {
            ROS_ERROR("Could not get wheel radius from parameter server.");
            return -1;
        }
        r = parameter;
        if (!node.getParam("parameter/initial/x", parameter)) {
            parameter = 0;
        }
        poseMobile.x = poseWorld.x  = parameter;
        if (!node.getParam("parameter/initial/y", parameter)) {
            parameter = 0;
        }
        poseMobile.y = poseWorld.y  = parameter;
        if (!node.getParam("parameter/initial/theta", parameter)) {
            parameter = 0;
        }
        poseMobile.theta = poseWorld.theta  = parameter;
    }
    ros::Subscriber subscriber_link_state;
    ros::Subscriber subscriber_wheel_vel;
    subscriber_link_state = node.subscribe("/gazebo/link_states", 1, onGazeboMessage);
    subscriber_wheel_vel = node.subscribe("sensor/wheel_velocity", 1, onEncoderMessage);

    
    kinematicsForwardWorld  = node.serviceClient<open_base::KinematicsForward>("kinematics_forward_world" );
    kinematicsForwardMobile = node.serviceClient<open_base::KinematicsForward>("kinematics_forward_mobile");
    publisherWorld  = node.advertise<geometry_msgs::Pose2D>("pose/world" , 1);
    publisherMobile = node.advertise<geometry_msgs::Pose2D>("pose/mobile", 1);
    ros::spin();
    return 0;
}
