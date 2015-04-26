#!/usr/bin/python

import rospy, math, my_utils
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

## @class ObsticleRanger
# A laser scanner "listener" that holds and constantly updates the closest point from the left and from the right of the scanner.
class ObsticleRanger:

    C_MIN_SCAN_ANGLE = math.radians(-60)
    C_MAX_SCAN_ANGLE = math.radians(60)

    ##
    # @param name - the string identifier of the robot for whom we want to parse the laser data.
    def __init__(self, name):
        self.name = name
        self.closest_left_point = self.closest_right_point = self.closest_point = 1
        rospy.Subscriber("/%s/base_scan" % self.name, LaserScan, self.laser_callback)
        
    ## The callback function that fills the nearest points values each time a base_scan message is recieved.
    def laser_callback(self, scan):
        min_index = int(math.ceil((self.C_MIN_SCAN_ANGLE - scan.angle_min) / scan.angle_increment))
        max_index = int(math.floor((self.C_MAX_SCAN_ANGLE - scan.angle_min) / scan.angle_increment))
        self.closest_left_point = min(scan.ranges[min_index : max_index/2])
        self.closest_right_point = min(scan.ranges[max_index/2 : max_index])
        self.closest_point = min(self.closest_left_point, self.closest_right_point)


## @class FormationRobot
# A base class for the Leader and Follower classes to inherit.
class FormationRobot(object):

    C_MIN_DIST_TO_AN_OBSTICLE = 0.5

    ##
    # @param name        - the string identifier of the robot.
    # @param leader_name - the string identifier of the robot that this robot should follow.
    #                      Note that this param has no meaning in the Leader class and will be ignored there.
    def __init__(self, name, leader_name):
        self.name = name
        self.leader_name = leader_name
        self.should_move = False
        self.rate = rospy.Rate(my_utils.C_DEFAULT_RATE)        
        self.obsticle_ranger = ObsticleRanger(self.name)
        self.publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)        

    ## The function that causes the robot to start moving.
    def start(self):
        self.should_move = True

    ## The function that causes the robot to stop moving.
    def stop(self):
        self.should_move = False
    

