class Pose{
private:
    double pos_x_;
    double pos_y_;
    double pos_theta_;

public:
    Pose(){
    }

    Pose(double x, double y, double theta)
    :pos_x_(x), pos_y_(y), pos_theta_(theta)
    {}

    double getX(){
        return pos_x_;
    }
    double getY(){
        return pos_y_;
    }
    double getTheta(){
        return pos_theta_;
    }

    void setPose(double x, double y, double theta){
        pos_x_ = x;
        pos_y_ = y;
        pos_theta_ = theta;
    }

};