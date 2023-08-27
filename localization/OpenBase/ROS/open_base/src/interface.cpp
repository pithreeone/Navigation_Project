#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <open_base/Movement.h>
#include <open_base/MovementBezier.h>
#include <open_base/MovementGeneric.h>
#include <open_base/Velocity.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64MultiArray.h>

#include <vector>

open_base::Movement movement_msgs;
double vx = 0.0;
double vy = 0.0;
double w = 0.0;

void velCallback(const geometry_msgs::Twist& twist){
  ROS_INFO("debug");
  vx = twist.linear.x;
  vy = twist.linear.y;
  w = twist.angular.z;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "open_base_interface");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("cmd_vel", 1000, velCallback);
  ros::Publisher pub = nh.advertise<open_base::Movement>("open_base/command", 1000);
  ros::Rate loop_rate(100);

  movement_msgs.movement = 1;
  movement_msgs.generic.type = 2; // for velocity (linear and angular)
  movement_msgs.generic.frame = 1; // mobile frame

  // gemetry_msgs::Pose p;
  // p.position.x = 0;
  // p.position.y = 0;
  // p.position.theta = 0;
  // movement_msgs.bezier.targetTranslation.insert(p);
  std::vector<double> rotation = {0.0};
  movement_msgs.bezier.targetRotation = rotation;



  while(ros::ok()){
    movement_msgs.generic.target.x = vx;
    movement_msgs.generic.target.y = vy;
    movement_msgs.generic.target.theta = w;
    pub.publish(movement_msgs);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}