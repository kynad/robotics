#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class Wonderer {
public:
    // Tunable parameters
    const static double FORWARD_SPEED_MPS = 0.5;
    const static double TURN_ANGLE = 0.5;
    const static double MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
    const static double MAX_SCAN_ANGLE_RAD = +30.0/180*M_PI;
    const static float DEFAULT_MIN_PROXIMITY_RANGE_M = 0.5; // Should be smaller than sensor_msgs::LaserScan::range_max

    Wonderer(float proximity);
    void start();

private:
    ros::NodeHandle node;
    ros::Publisher commandPub; // Publisher to the robot's velocity command topic
    ros::Subscriber laserSub; // Subscriber to the robot's laser scan topic
    float minProximityRange;
    float minLeftRange;
    float minRightRange;

    void moveForward();
    void turn(int direction);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    float findMinimalRange(const float* first, const float* last, float allowedMinRange, float allowedMaxRange);
    bool canMove();
    void avoidObstacles();
    void sendMoveCommand(double linear, double angular);
    
};

