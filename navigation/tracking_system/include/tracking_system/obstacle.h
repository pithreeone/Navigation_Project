#include <tf/transform_listener.h>
#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Dense>

class Obstacle{
    private:
        // Pose
        double x_;
        double y_;

        // Velocity
        double vx_;
        double vy_;

        // KF-state
        Eigen::Matrix<double, 4, 1> state;
        Eigen::Matrix<double, 4, 4> cov_mat;
        Eigen::Matrix<double, 4, 4> A;
        Eigen::Matrix<double, 2, 4> C;
        Eigen::Matrix<double, 4, 4> Q;
        Eigen::Matrix<double, 2, 2> R;


        double dt_;

        int id_;

        ros::Time stamp_;
    public:
        Obstacle();

        Obstacle(double x, double y, double dt, int id, ros::Time stamp);

        void setPose(double x, double y);

        void getXY(double &x, double &y){
            x = x_;
            y = y_;
        };

        ros::Time getStamped(){
            return stamp_;
        }

        void setStamped(ros::Time stamp){
            stamp_ = stamp;
        }

        double distanceBetween(double x, double y);

        void KF_predict();

        void KF_update(double x, double y);

        void printData();

        void printState();
};

class Obstacles_handler{
    private: 
        ros::NodeHandle nh_;
        ros::NodeHandle nh_local_;
        tf::TransformListener tf_listener_;
        ros::Subscriber sub_obstacles_;
        ros::Subscriber sub_target_pose_;
        ros::Publisher pub_target_;
        ros::Publisher pub_target_pose_;
        ros::Publisher pub_obstacles_filtered_;
        ros::Publisher pub_obstacles_array_filtered_;
        
        std::vector<Obstacle> obstacles_;
        double dist_thres_;
        double dt_;

        Obstacle target_obstacle_;
    public:

        Obstacles_handler(ros::NodeHandle& nh, ros::NodeHandle& nh_local);

        void initializeParam();

        void initialize();

        void targetCallback(const obstacle_detector::Obstacles::ConstPtr& msg);

        void setTargetCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

        geometry_msgs::PointStamped obstacle_frame_transform(std::string original_frame, std::string target_frame, double x, double y);

        int checkIfOldObstacle(double x, double y);

        void pushObstacle(double x, double y);

        void predictObstacle();

        void updateObstacle(int id, double x, double y);

        void findNearestObstacleAndPublish();

        void clearOldObstacle();

        void visualizeObstacleFiltered();

};