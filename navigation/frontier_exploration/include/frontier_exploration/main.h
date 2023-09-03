#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_listener.h>
#include <queue>
#include <frontier_exploration/map.h>


class Frontier_Exploration{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_local_;
        ros::Subscriber sub_map_;
        ros::Subscriber sub_pose_;
        ros::Publisher pub_goal_;
        ros::Publisher pub_frontier_map_;
        ros::Publisher pub_frontier_groups_map_;
        nav_msgs::OccupancyGrid map_;
        nav_msgs::OccupancyGrid map_buffer_;
        Map origin_map_; 
        bool if_map_updated_;


        // ros parameters
        std::string map_frame_;
        std::string base_frame_;
        int thresh_;
        int clear_frontiers_;

        // tf for getting base_footprint to map
        tf::TransformListener listener_;
        tf::StampedTransform transform_;
    
        double cur_pose_x_;
        double cur_pose_y_;
        double cur_pose_theta_;

    public:
        int n_frontier_group_;
        std::queue <Frontier> frontier_points_;
        
        Frontier_Exploration(ros::NodeHandle& nh, ros::NodeHandle& nh_local){
            nh_ = nh;
            nh_local_ = nh_local;
        }
        void initialize();

        /// @brief get parameter from yaml file
        void initializeParam();

        /// @brief get robot psoe from tf (map->base_footprint)
        void getRobotPose();

        void copyGridMap(nav_msgs::OccupancyGrid map, nav_msgs::OccupancyGrid& map_copy);

        void publishFrontierMap();
        void WFD();
        /// @brief publish map type with different groups of frontier with different color 
        void publishFrontierGroupsMap();
        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

};