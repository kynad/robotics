#!/usr/bin/python

import rospy, tf, math

def distance(point1, point2):
    delta = pow(point2[0]-point1[0],2)+pow(point2[1]-point1[1],2)
    return math.sqrt(delta)
    

class TransformListenerWrapper:

    NUM_RETRIES = 3

    def __init__(self, dest_frame, target_frame, timeout=5.0):
        self.dest_frame = dest_frame
        self.target_frame = target_frame
        self.timeout = rospy.Duration(timeout)
        self.listener = tf.TransformListener()
        self.wait_for_frames()
        
    def wait_for_frames(self):
        rospy.sleep(1)
        self.listener.waitForTransform(self.dest_frame, self.target_frame, rospy.Time.now(), self.timeout)
        
    def get_coordinates(self):
        coords = None
        for i in range(self.NUM_RETRIES):
            try:
                (coords, rot) = self.listener.lookupTransform(self.dest_frame, self.target_frame, rospy.Time.now())
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception) as err:
                rospy.logwarn("Failed in transform lookup (from %s to %s) due to the following reason: %s" % (self.dest_frame, self.target_frame, str(err)))
                rospy.sleep(0.1)
        if coords == None:
            raise err
        return coords
        
    def get_2d_coordinates(self):
        return self.get_coordinates()[:2]
