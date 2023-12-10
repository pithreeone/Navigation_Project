#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


#define PI 3.1415926

double target_x;
double target_y;
double target_yaw;
double target_direction;  // 0: 0deg, 1: 90deg, 2: 180deg, 3: 270deg

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

    double maximum_velocity = 0.15;
    double maximum_angular_velocity = 0.7;
    vx = maximum_velocity * target_x / sqrt(target_x * target_x + target_y * target_y);
    vy = maximum_velocity * target_y / sqrt(target_x * target_x + target_y * target_y);


    // angular_kp
    double angular_kp = 6;
    if((0 < target_direction && target_direction <= 0.5) || (1 < target_direction && target_direction <= 1.5) || (-2 < target_direction && target_direction <= -1.5) || (-1 < target_direction && target_direction <= -0.5)){
        w = angular_kp * fmod((target_direction + 2), 1);
    }else if((0.5 < target_direction && target_direction <= 1) || (1.5 < target_direction && target_direction <= 2) || (-1.5 < target_direction && target_direction <= -1) || (-0.5 < target_direction && target_direction <= 0)){
        w = angular_kp * fmod((target_direction - 2), 1);
    }
    
    // hard-constraint
    if(w >= maximum_angular_velocity){
        w = maximum_angular_velocity;
    }else if(w <= -maximum_angular_velocity){
        w = -maximum_angular_velocity;
    }

    // tolerance
    double angle_tolerance = 0.1;     // 0.1 ~= 6 deg
    if((fabs(target_direction - (-2)) *PI/2  <= angle_tolerance) || (fabs(target_direction - (-1))*PI/2 <= angle_tolerance*PI/2)
    || (fabs(target_direction - (0)) *PI/2 <= angle_tolerance*PI/2) || (fabs(target_direction - (1))*PI/2 <= angle_tolerance*PI/2)
    || (fabs(target_direction - (2)) *PI/2 <= angle_tolerance*PI/2))
    {
        w = 0;
    }

    double stop_distance = 1.0;
    if(dist <= stop_distance){
        vx = 0;
        vy = 0;
    }
}

void publishVelocity(ros::Publisher pub, double vx, double vy, double w){
    double low_pass_filter_gain = 0.1;

    static double pre_vx, pre_vy, pre_w;
    geometry_msgs::Twist vel;
    pre_vx = vel.linear.x = pre_vx + low_pass_filter_gain * (vx - pre_vx);
    pre_vy = vel.linear.y = pre_vy + low_pass_filter_gain * (vy - pre_vy);
    pre_w = vel.angular.z = pre_w + low_pass_filter_gain * (w - pre_w);

    pub.publish(vel);

}

void getTransform(tf2_ros::Buffer& tfBuffer){
    // Transform the coordinate to the target frame
    try {
        tfBuffer.canTransform("base_scan", "base_link",
                                ros::Time(0), ros::Duration(1.0));
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("base_scan", "base_link", ros::Time(0));
        
        // Extract yaw from the quaternion
        tf2::Quaternion quat;
        tf2::fromMsg(transformStamped.transform.rotation, quat);
        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        ROS_INFO("base_scan -> base_link: %f", yaw);
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("Transform failed: %s", ex.what());
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "simple_cpp_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    ros::Subscriber sub_tracking_target = nh.subscribe("tracking_target", 1000, targetCallback);
    ros::Subscriber sub_tracking_target_stamped = nh.subscribe("destination_pose", 1000, targetStampedCallback);
    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    getTransform(tfBuffer);

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