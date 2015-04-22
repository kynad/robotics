#!/usr/bin/python

import rospy, math, my_utils
from geometry_msgs.msg import Twist


class Follower:

    C_MIN_DIST = 0.8
    C_MAX_LINEAR_VEL = 0.7
    C_MAX_ANGULAR_VEL = 3.14

    C_FRAME_STR = "/robot_%s/base_footprint"
    
    def __init__(self, my_id,):
        self.my_id = my_id
        self.should_follow = False
        self.publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)        

    def follow(self, leader_id):
        self.should_follow = True
        rate = rospy.Rate(10.0)
        tf_listener = my_utils.TransformListenerWrapper(self.C_FRAME_STR % self.my_id, self.C_FRAME_STR % leader_id)
        while self.should_follow and not rospy.is_shutdown():
            coords = tf_listener.get_coordinates()
            dist_from_leader = my_utils.distance(coords, (0,0))
            velocity_msg = Twist()
            if dist_from_leader > self.C_MIN_DIST:
                velocity_msg.linear.x = min(0.5 * dist_from_leader, self.C_MAX_LINEAR_VEL)
                velocity_msg.angular.z = min(4 * math.atan2(coords[1], coords[0]), self.C_MAX_ANGULAR_VEL)
            self.publisher.publish(velocity_msg)
            rate.sleep()
            
    def stop_following(self):
        self.should_follow = False
    

