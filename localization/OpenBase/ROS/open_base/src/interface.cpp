#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <open_base/Movement.h>
#include <open_base/MovementBezier.h>
#include <open_base/MovementGeneric.h>
#include <open_base/Velocity.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

open_base::Movement movement_msgs;
double x = 0.0;
double y = 0.0;
double theta = 0.0;
geometry_msgs::Quaternion quat;

double vx = 0.0;
double vy = 0.0;
double w = 0.0;

void velCallback(const geometry_msgs::Twist& twist){
  vx = twist.linear.x;
  vy = twist.linear.y;
  w = twist.angular.z;
}

void poseCallback(const geometry_msgs::Pose2D& pose){
  x = pose.x;
  y = pose.y;
  theta = pose.theta;

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = "base_footprint";
  transformStamped.transform.translation.x = x;
  transformStamped.transform.translation.y = y;
  transformStamped.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  quat = tf2::toMsg(q);
  br.sendTransform(transformStamped);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "interface");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("cmd_vel", 1000, velCallback);
  ros::Subscriber sub_pose = nh.subscribe("pose/mobile", 10, poseCallback);
  ros::Publisher pub = nh.advertise<open_base::Movement>("command", 1000);
  ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Rate loop_rate(100);

  movement_msgs.movement = 1;
  movement_msgs.generic.type = 2; // for velocity (linear and angular)
  movement_msgs.generic.frame = 1; // mobile frame

  std::vector<double> rotation = {0.0};
  movement_msgs.bezier.targetRotation = rotation;



  while(ros::ok()){
    // publish open_base::Movement
    movement_msgs.generic.target.x = vx;
    movement_msgs.generic.target.y = vy;
    movement_msgs.generic.target.theta = w;
    pub.publish(movement_msgs);

    // publish odometry message
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";

    // set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = quat;
    // set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = w;

    pub_odom.publish(odom);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}