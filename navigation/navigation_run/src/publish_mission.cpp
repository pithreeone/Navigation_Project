#include <ros/ros.h>
#include <robot_interface/Interface.h>

void printHint(){
    std::cout << "Choose a mission to publish: " << std::endl;
    std::cout << "[1]: start_mapping" << std::endl;
    std::cout << "[2]: finish_control_mapping" << std::endl;
    std::cout << "[3]: check_map" << std::endl;
}


int main(int argc, char** argv){
    
    ros::init(argc, argv, "test_program");
    ros::NodeHandle nh;
    int mission;
    ros::Publisher pub_action = nh.advertise<robot_interface::Interface>("action", 10);

    int n_action = 0;
    while(ros::ok()){
        std::cout << "*-----------------------------" << n_action++ << "-----------------------------*" << std::endl;
        printHint();
        std::cout << "your mission(please type the corresponding number): ";
        std::cin >> mission;
        robot_interface::Interface action;
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
        }
        pub_action.publish(action);
        ros::spinOnce();
    }
    

    return 0;
}