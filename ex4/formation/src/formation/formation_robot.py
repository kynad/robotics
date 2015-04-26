#!/usr/bin/python

from formation_robot_base import *
import rospy, math, my_utils
from random import randint
from geometry_msgs.msg import Twist

## @class Follower
#  A FormationRobot class that implements the follower behavior.
class Follower(FormationRobot):

    C_FRAME_STR = "/%s/base_footprint"
    C_MIN_DIST_TO_LEADER = 0.8
    C_MAX_LINEAR_VEL = 0.7
    C_MAX_ANGULAR_VEL = math.pi

    ## Will start moving if and when its immediate leader is too far away trying to stay on its tail.
    def start(self):
        super(self.__class__, self).start()
        tf_listener = my_utils.TransformListenerWrapper(self.C_FRAME_STR % self.name, self.C_FRAME_STR % self.leader_name)
        while self.should_move and not rospy.is_shutdown():
            coords = tf_listener.get_2d_coordinates()
            dist_from_leader = my_utils.distance(coords, (0,0))
            velocity_msg = Twist()
            if dist_from_leader > self.C_MIN_DIST_TO_LEADER:
                if self.obsticle_ranger.closest_point > self.C_MIN_DIST_TO_AN_OBSTICLE:
                    velocity_msg.linear.x = min(0.5 * dist_from_leader, self.C_MAX_LINEAR_VEL)                
                    velocity_msg.angular.z = min(4 * math.atan2(coords[1], coords[0]), self.C_MAX_ANGULAR_VEL)
                else:
                    velocity_msg.angular.z = 0.5 if self.obsticle_ranger.closest_left_point < self.obsticle_ranger.closest_right_point else -0.5
            self.publisher.publish(velocity_msg)
            self.rate.sleep()
            

## @class Leader
#  A FormationRobot class that implements the random walk behavior.
class Leader(FormationRobot):

    C_LINEAR_SPEED = 0.5
    C_ANGULAR_SPEED = 0.5
    
    ## Will start moving forward until hitting an obsticle.
    #  If an obsticle is too close, will start turning to a random direction, until stops seeing an object closer than a certain treshold.
    def start(self):
        super(self.__class__, self).start()
        linear_twist = Twist()
        linear_twist.linear.x = self.C_LINEAR_SPEED
        angular_twist = Twist()
        angular_twist.angular.z = self.C_ANGULAR_SPEED
        
        while self.should_move and not rospy.is_shutdown():
            if self.obsticle_ranger.closest_point > self.C_MIN_DIST_TO_AN_OBSTICLE:
                self.publisher.publish(linear_twist)
                random_direction = 2 * randint(0,1) - 1
                angular_twist.angular.z *= random_direction
            else:
                self.publisher.publish(angular_twist)
            self.rate.sleep()
        
    

