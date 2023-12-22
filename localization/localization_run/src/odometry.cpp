#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <string.h>

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

void velCallback(const geometry_msgs::Twist& data){
    vx = data.linear.x;
    vy = data.linear.y;
    vth = data.angular.z;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::NodeHandle nh_local("~");
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber odom_sub = n.subscribe("Toposition", 1000, velCallback);
  std::string frame_id;
  std::string child_frame_id;
  tf::TransformBroadcaster odom_broadcaster;
  double rate;
  double angle_multiplier;
  
  nh_local.param<std::string>("odom_frame", frame_id, "odom");
  nh_local.param<std::string>("base_frame", child_frame_id, "base_link");
  nh_local.param<double>("rate", rate, 100);
  nh_local.param<double>("angle_multiplier", angle_multiplier, 1.0);

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(rate);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * angle_multiplier * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id = child_frame_id;

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = frame_id;

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = child_frame_id;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}