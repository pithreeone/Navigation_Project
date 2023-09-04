#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_listener.h>
#include <queue>
#include <frontier_exploration/map.h>
enum Robot_State{
    Explore,
    Stop,
    Arrive,
};

class Frontier_Exploration{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_local_;
        ros::Subscriber sub_map_;
        ros::Subscriber sub_pose_;
        ros::Subscriber sub_odom_feedback_;
        ros::Publisher pub_goal_;
        ros::Publisher pub_frontier_map_;
        ros::Publisher pub_frontier_groups_map_;
        ros::Publisher pub_frontier_center_map_;
        nav_msgs::OccupancyGrid map_;
        nav_msgs::OccupancyGrid map_buffer_;
        Map origin_map_; 
        bool if_map_updated_;
        Robot_State state_;
        Pose cur_pose_;
        std::vector<Frontier> frontier_centers_;

        // ros parameters
        std::string map_frame_;
        std::string base_frame_;
        int thresh_;
        int clear_frontiers_;

        // tf for getting base_footprint to map
        tf::TransformListener listener_;
        tf::StampedTransform transform_;

    public:
        int n_frontier_group_;
        std::vector<Frontier> frontier_points_;
        
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

        /// @brief if groups of frontier is smaller than clear_frontiers_, then delete.
        void deleteSmallFrontier();

        /// @brief calculate nearest frontiers center
        /// @param p the answer will store in this argument
        /// @return if no any frontier return false
        bool calculateNearestFrontier(Pose& p);

        /// @brief publish frontier points which is in frontier_points_ 
        void publishFrontierGroupsMap();

        void publishFrontierCenterMap();

        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        void odomFeedbackCallback(const std_msgs::Char::ConstPtr& msg);

};