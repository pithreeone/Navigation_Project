#include <ros/ros.h>
#include <pme_amr_msg/Interface.h>
#include <tf/transform_broadcaster.h>

#define PI 3.1415926

void printHint(){
    std::cout << "Choose a mission to publish: " << std::endl;
    std::cout << "---------- slam-mode ----------" << std::endl;
    std::cout << "[1]: start_mapping" << std::endl;
    std::cout << "[2]: finish_control_mapping" << std::endl;
    std::cout << "[3]: check_map" << std::endl;
    std::cout << "---------- navigation-mode ----------" << std::endl;
    std::cout << "[4]: move_to_goal" << std::endl;
    std::cout << "[5]: choose_map" << std::endl;
    std::cout << "[6]: move_to_goal_floor" << std::endl;
    std::cout << "[7]: DEBUG" << std::endl;
}


int main(int argc, char** argv){
    
    ros::init(argc, argv, "test_program");
    ros::NodeHandle nh;
    int mission;
    ros::Publisher pub_action = nh.advertise<pme_amr_msg::Interface>("action", 1);

    int n_action = 0;
    while(ros::ok()){
        std::cout << "*-----------------------------" << n_action++ << "-----------------------------*" << std::endl;
        printHint();
        std::cout << "your mission(please type the corresponding number): ";
        std::cin >> mission;
        pme_amr_msg::Interface action;
        switch(mission){
            case 1:{
                action.mission = "start_mapping";
                break;
            }
            case 2:{
                action.mission = "finish_control_mapping";
                break;
            }
            case 3:{
                action.mission = "check_map";
                break;
            }
            case 4:{
                action.mission = "move_goal";
                std::cout << "please type the coordinate [x (m), y (m), theta (degree)]: ";
                double x, y, theta;
                std::cin >> x >> y >> theta;
                action.goal.header.stamp = ros::Time::now();
                action.goal.header.frame_id = "map";
                action.goal.pose.position.x = x;
                action.goal.pose.position.y = y;
                tf::Quaternion q;
                q.setRPY(0, 0, theta/180.0*PI);
                geometry_msgs::Quaternion odom_quat;
                tf::quaternionTFToMsg(q, odom_quat);
                action.goal.pose.orientation = odom_quat;
                break;
            }
            case 5:{
                action.mission="choose_map";
                std::cout << "please type the floor, just type the number: ";
                std::string floor;
                std::cin >> floor;
                std::string map_name = "EngBuild" + floor;
                action.choose_map_name = map_name;
                break;
            }
            case 6:{
                action.mission="move_to_goal_floor";
                std::cout << "please type the floor, just type the number: ";
                std::string floor;
                std::cin >> floor;
                int8_t floor_int8 = static_cast<int8_t>(stoi(floor));
                action.floor.data = floor_int8;
                std::cout << "please type the coordinate [x (m), y (m), theta (rad)]: ";
                double x, y, theta;
                std::cin >> x >> y >> theta;
                action.goal.header.stamp = ros::Time::now();
                action.goal.header.frame_id = "map";
                action.goal.pose.position.x = x;
                action.goal.pose.position.y = y;
                tf::Quaternion q;
                q.setRPY(0, 0, theta);
                geometry_msgs::Quaternion odom_quat;
                tf::quaternionTFToMsg(q, odom_quat);
                action.goal.pose.orientation = odom_quat;
                break;
            }
            case 7:{
                action.mission="debug";
            }
        }
        pub_action.publish(action);
        ros::spinOnce();
    }
    

    return 0;
}