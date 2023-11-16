/*
 * altitude2floor
 * 2023, Nov, 6
 *
 * Description:
 * The code is to transfer the pressure to floor number. Including Initialize, Calibration, Reintialize ...
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <std_msgs/Int8.h>

/**
 * For initialize and calibration variables
 */
double pressure_init = 0;
bool if_initialized = false;          // if need to re-initialize. just set to false
int cali_n = 100;
int n = 0;

/**
 * Need 3 parameters to do the transform.
 * p_ideal = p0 + (floor_init - floor) * delta_p
 */
double p0 = 0;
int floor_init = 3;
double delta_p = 0.4;
double error = 0.2; 

/**
 * current pressure & floor 
 */
double pressure_cur = 0;
int floor_cur = 0;

void pressureCallback(const sensor_msgs::FluidPressure& data){
    pressure_cur = data.fluid_pressure;

    // calculate the first $cali_n data and take average
    if(!if_initialized){
        if(n++ < cali_n){
            p0 += pressure_cur;
            ROS_INFO_THROTTLE(0.5, "[altitude2floor]: Do calibration ...  %.2f %%", (double)n / cali_n);
        }else{
            p0 /= cali_n;
            if_initialized = true;
            ROS_INFO("[altitude2floor]: average pressure is: %.3fhPa", p0);
        }
        return;
    }

    // calculate floor
    if(if_initialized){
        bool success_mapping = false;
        for(int i=1; i<=6; i++){
            double ideal_p = p0 + (floor_init - i) * delta_p;
            if(fabs(ideal_p - pressure_cur) < error){
                floor_cur = i;
                success_mapping = true;
                break;
            }
        }
        if(!success_mapping){
            ROS_WARN("[altitude2floor]: Cannot mapping from pressure to floor! May need calibration.");
        }else{
            ROS_INFO_THROTTLE(0.5, "[altitude2floor]: transform successfully. Floor:%d", floor_cur);
        }
    }
    
}

void floorCaliCallback(const std_msgs::Int8& data){
    floor_init = floor_cur = data.data;
    if_initialized = false;
    p0 = 0;
    n = 0;
    ROS_INFO("[altitude2floor]: Successfully reinitialize. Do calibration again.");
}


int main(int argc, char** argv){
    ros::init(argc, argv, "floor_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    ros::Publisher floor_pub = nh.advertise<std_msgs::Int8>("floor", 10);
    ros::Subscriber floor_cali_sub = nh.subscribe("floor_calibration", 10, floorCaliCallback);
    ros::Subscriber pressure_sub = nh.subscribe("bmp280/pressure", 50, pressureCallback);
    ros::Rate r(50);

    while(nh.ok()){
        if(if_initialized){
            std_msgs::Int8 pub_data;
            pub_data.data = floor_cur;
            floor_pub.publish(pub_data);
        }
        r.sleep();
        ros::spinOnce();
    }

}