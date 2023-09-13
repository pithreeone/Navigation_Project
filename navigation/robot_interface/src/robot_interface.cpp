#include "ros/ros.h"
#include "robot_interface/fsm_item.h"
#include "robot_interface/fsm.h"

void Interface::initialize()
{
    
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_interface");
    ros::NodeHandle nh, nh_local("~");

    Interface interface(nh, nh_local);

    return 0;
}