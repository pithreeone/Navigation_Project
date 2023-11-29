
class Obstacle{
    private:
        // Pose
        double x_;
        double y_;
        double yaw_;
        // Velocity
        double vx_;
        double vy_;
        double w_;

        double dt_;

        int id_;
    public:
        Obstacle();

        Obstacle(double x, double y, double dt, int id);

        void setPose(double x, double y);

        double distanceBetween(double x, double y);


        void printData();
};

class Obstacles_handler{
    private: 
        std::vector<Obstacle> obstacles_;
        double dist_thres_;
    
    public:
        Obstacles_handler();

        bool check_if_old_obstacle(double x, double y);


};