#include "nav_msgs/OccupancyGrid.h"
#include <queue>
#include <ros/ros.h>
enum class Point_Indicator{
    Map_Open_List,
    Map_Close_List,
    Frontier_Open_List,
    Frontier_Close_List,
    UDF,
};

enum class Direction{
    Front,
    Left,
    Back,
    Right,
    Front_Left,
    Back_Left,
    Back_Right,
    Front_Right
};

class Pose{
    public:
        double x_;
        double y_;
        double theta_;
        Pose(){

        }

        Pose(double x, double y, double theta){
            x_ = x; y_ = y; theta_ = theta;
        }
};

class Frontier: public Pose{
    public:
        int id_;
        Frontier(){
        }
        Frontier(Pose p, int id){
            x_ = p.x_; y_ = p.y_; id_ = id;
        }
};

class Map{
    nav_msgs::OccupancyGrid grid_map_;
    Point_Indicator **indicator_map_;
    double res_;
    double thresh_;

    public:

        Map(){
        }

        Map(nav_msgs::OccupancyGrid map, int thresh);
        
        void setMap(nav_msgs::OccupancyGrid map){
            grid_map_ = map;
            res_ = map.info.resolution;
            // memory allocation
            indicator_map_ = new Point_Indicator*[getWidth()];
            for(int i=0; i<getWidth(); i++){
                indicator_map_[i] = new Point_Indicator[getHeight()];
            }
            for(int i=0; i<getWidth(); i++){
                for(int j=0;j<getHeight(); j++){
                    indicator_map_[i][j] = Point_Indicator::UDF;
                }
            }
        }

        void setThresh(int thresh){
            thresh_ = thresh;
        }

        int getWidth(){
            return grid_map_.info.width;
        }

        int getHeight(){
            return grid_map_.info.height;
        }
        double getResolution(){
            return res_;
        }

        void worldToMap(Pose p, int& idx, int& idy){
            if(res_ == 0 ){
                ROS_WARN("Frontier_exploration: resolution is 0");
                return;
            }
            idx = int((p.x_ - grid_map_.info.origin.position.x) / res_);
            idy = int((p.y_ - grid_map_.info.origin.position.y) / res_);
        }

        void worldToMap(Frontier p, int& idx, int& idy){
            if(res_ == 0 ){
                ROS_WARN("Frontier_exploration: resolution is 0");
                return;
            }
            idx = int((p.x_ - grid_map_.info.origin.position.x) / res_);
            idy = int((p.y_ - grid_map_.info.origin.position.y) / res_);
        }

        void mapToWorld(int idx, int idy, Pose &p){
            p.x_ = grid_map_.info.origin.position.x + idx * res_;
            p.y_ = grid_map_.info.origin.position.y + idy * res_;
        }

        int getCellData(int idx, int idy){
            return grid_map_.data[idy * grid_map_.info.width + idx];
        }
        
        /// @brief get neighbor data of map
        /// @param idx 
        /// @param idy 
        /// @param dir four direction, can fill with "up" "down" "left" "right"
        /// @return if the neighbor exceed map, return -2
        int getNeighborData(int idx, int idy, Direction dir);

        /// @brief check whether coordinate(idx, idy) is frontier point
        /// @param idx 
        /// @param idy 
        /// @return if frontier point return true, otherwise return false
        bool ifFrontierPoint(int idx, int idy);

        
        /// @brief check whether coordinate(idx, idy) is open-space point
        /// @param idx 
        /// @param idy 
        /// @return 
        bool ifOpenSpacePoint(int idx, int idy);

        void setIndicatorData(int idx, int idy, Point_Indicator ind){
            indicator_map_[idx][idy] = ind;
        }

        Point_Indicator getIndicatorData(int idx, int idy){
            return indicator_map_[idx][idy];
        }


        /// @brief get neighbor indicator of indicator_map
        /// @param idx 
        /// @param idy 
        /// @param dir choose one direction
        /// @return no 
        Point_Indicator getNeighborIndicator(int idx, int idy, Direction dir);

        /// @brief in pseudo-code 27, check v has at least one "Map-Open_Space" neighbot
        /// @param idx 
        /// @param idy 
        /// @return 
        bool ifHasMapOpenSpaceNeighbor(int idx, int idy);
};