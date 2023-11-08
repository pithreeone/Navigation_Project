#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>

#include <iostream>
#include <fstream> // Include this header
#include <vector>
#include <string>

// void floorCaliCallback(const std_msgs::Int8& data){
//     ROS_INFO("[altitude2floor]: Successfully reinitialize. Do calibration again.");
// }

bool catch_buf = false;
sensor_msgs::LaserScan scan_buf;
int data_n = 0;
int n = 360;


// Sample 2D array (matrix) to write to the CSV file
std::vector<std::vector<double>> matrix;
void laserScanCB(const sensor_msgs::LaserScan& data){
    scan_buf = data;
}
void catchCB(const std_msgs::Bool& data){
    catch_buf = data.data;
}

void writeFile(std::string filename){
    // Open the file for writing
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename << " for writing." << std::endl;
        return;
    }

    // add the title
    
    for(int i=0;i<n;i++){
        file << i;
        if (i < n - 1) {
            file << ",";
        }
    }
    file << "\n";

    // Write the matrix to the CSV file
    for (const auto& row : matrix) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];
            if (i < row.size() - 1) {
                file << ",";
            }
        }
        file << "\n";  // Start a new line for the next row
    }

    // Close the file
    file.close();

    std::cout << "CSV file " << filename << " has been created." << std::endl;

}

int main(int argc, char** argv){
    ros::init(argc, argv, "floor_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");

    std::string filename;
    nh_local.param<std::string>("path", filename, "none");
    // ros::Publisher floor_pub = nh.advertise<std_msgs::Int8>("floor", 10);
    ros::Subscriber scan_sub = nh.subscribe("scan", 10, laserScanCB);
    ros::Subscriber catch_data_sub = nh.subscribe("catch", 10, catchCB);
    ros::Rate r(50);

    while(nh.ok()){
        if(catch_buf){
            std::vector<double> temp;
            for(int i=0;i<n;i++){
                temp.push_back(scan_buf.ranges[i]);
            }
            matrix.push_back(temp);
            data_n++;
            ROS_INFO("Successfully save a data. Number of data: %d", data_n);
            catch_buf = false;
        }
        r.sleep();
        ros::spinOnce();
    }
    writeFile(filename);

}