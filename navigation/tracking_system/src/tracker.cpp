#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>

double target_x;
double target_y;
double target_yaw;

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
}

void trackingController(double& vx, double& vy, double& w){
    double dist = std::sqrt(target_x * target_x + target_y * target_y);
    // ROS_INFO("target_x: %f, target_y: %f", target_x, target_y);
    if(dist <= 1.0){
        vx = 0;
        vy = 0;
        w = 0;
        return;
    }
    double v = 0.2;
    vx = 0.2 * target_x / sqrt(target_x * target_x + target_y * target_y);
    vy = 0.2 * target_y / sqrt(target_x * target_x + target_y * target_y);
    w = 0;
    
}

void publishVelocity(ros::Publisher pub, double vx, double vy, double w){
    geometry_msgs::Twist vel;
    vel.linear.x = vx;
    vel.linear.y = vy;
    vel.angular.z = w;
    pub.publish(vel);

}

int main(int argc, char** argv){
    ros::init(argc, argv, "simple_cpp_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    ros::Subscriber sub_tracking_target = nh.subscribe("tracking_target", 1000, targetCallback);
    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    
    ros::Rate r(100);
    while(ros::ok()){
        double vx, vy, w;
        vx = vy = w = 0;
        trackingController(vx, vy, w);
        publishVelocity(pub_vel, vx, vy, w);
        // ROS_INFO_THROTTLE(0.2, "vx: %f,vy: %f,w: %f", vx, vy, w);
        r.sleep();
        ros::spinOnce();
    }

}