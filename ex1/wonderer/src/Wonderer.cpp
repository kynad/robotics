#include "Wonderer.h"
#include "geometry_msgs/Twist.h"

#define MIN(a,b) ((a < b) ? a : b)

Wonderer::Wonderer(float proximity)
{
    minProximityRange = DEFAULT_MIN_PROXIMITY_RANGE_M;
    if (proximity > 0.0) {
        minProximityRange = proximity;
    }
    minLeftRange = minProximityRange + 1.0;
    minRightRange = minProximityRange + 1.0;

    // Advertise a new publisher for the simulated robot's velocity command topic
    commandPub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // Subscribe to the simulated robot's laser scan topic
    laserSub = node.subscribe("base_scan", 1, &Wonderer::scanCallback, this);
}

// Send a velocity command
void Wonderer::sendMoveCommand(double linear, double angular) {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = linear;
    msg.angular.z = angular;
    commandPub.publish(msg);
}

// send a velocity command with linear sped only
void Wonderer::moveForward() {
    sendMoveCommand(FORWARD_SPEED_MPS, 0.0);
}

// send a velocity command with angular sped only
void Wonderer::turn(int direction) {
    sendMoveCommand(0.0, (double)direction * TURN_ANGLE);
}

// choose the diretion to turn to and call the turn function with that direction
void Wonderer::avoidObstacles() {
    int direction = -1;
    if (minRightRange < minLeftRange) {
        direction = 1;
    }
    turn(direction);
}

// return false if there is an obsticle too close and true otherwise
bool Wonderer::canMove() {
    float minRange = MIN(minLeftRange, minRightRange);
    return (minRange > minProximityRange);
}

// Process the incoming laser scan message to find the range to nearest obsticle from the left and from the right
void Wonderer::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
   int midIndex = (maxIndex - minIndex) / 2;

    const float* rangesPtr = &(scan->ranges[0]);

    minLeftRange = findMinimalRange(rangesPtr + minIndex, rangesPtr + midIndex, scan->range_min, scan->range_max);
    minRightRange = findMinimalRange(rangesPtr + midIndex + 1, rangesPtr + maxIndex, scan->range_min, scan->range_max);
}

// find the minimal range in a given slice of ranges, ignoring illigal values
float Wonderer::findMinimalRange(const float* first, const float* last, float allowedMinRange, float allowedMaxRange) {
    float minimalRange = *first;
    while (++first < last) {
        if (*first < allowedMinRange || *first > allowedMaxRange) {
            continue; // ignore values that are below rangeMin or above rangeMax
        }
        minimalRange = MIN(*first, minimalRange);
    }
    return minimalRange;
}

// main loop
void Wonderer::start()
{
    ros::Rate rate(10);

    // Keep spinning loop until user presses Ctrl+C
    while (ros::ok()) {
        if (canMove()) {
            moveForward();
        }
        else {
            avoidObstacles();
        }
        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
        rate.sleep();
    }
}

