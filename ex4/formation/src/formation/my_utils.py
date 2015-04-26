#!/usr/bin/python

import rospy, tf, math

C_DEFAULT_RATE = 10
C_MONITOR_TOPIC_NAME = "/monitor_topic"
C_ACTION_SERVER_STR = "/start_following_%s"

## Finds the 2D euclidean distance between two points.
#
# @param point1 - the (x,y) tuple, that are the coordinates of the first point.
# @param point2 - the (x,y) tuple, that are the coordinates of the second point.
#
# @return the distance between point1 and point2
def distance(point1, point2):
    delta = pow(point2[0]-point1[0],2)+pow(point2[1]-point1[1],2)
    return math.sqrt(delta)

## Calls a given function with the given arguments and retries if an exception was raised, waiting for a given amount of seconds (could be a fraction) between retries.
#
# @return a tuple consisting of the result of the function call (possibly None) and the last exception that was raised.    
def retry_loop(function, func_args, num_retries=3, sleep_between_retries=0.1):
    result = err = None
    for i in range(num_retries):
        try:
            result = function(*func_args)
        except Exception as err:
            rospy.logwarn("Got exception while executing %s, retrying (attempt %d out of %d)" % (function.__name__, i+1, num_retries))
            rospy.logwarn("Exception was: %s" % str(err))
            rospy.sleep(sleep_between_retries)
    return result, err
    

## @class TransformListenerWrapper
#  Wrapper around tf.TransformListener. Constructs such listener and supports calling to waitForTransformation() and lookupTransformation() methods.
class TransformListenerWrapper:
    ##
    # @param dest_frame   - the destination frame of the trnasformation
    # @param target_frame - the target frame of the trnasformation
    # @param timeout      - the timeout parameter that will be passed to waitForTransform method as rospy.Duration()
    def __init__(self, dest_frame, target_frame, timeout=5.0):
        self.dest_frame = dest_frame
        self.target_frame = target_frame
        self.timeout = rospy.Duration(timeout)
        self.listener = tf.TransformListener()
        rospy.sleep(1)        
        self.wait_for_frames()
        
    ## Wrapper around waitForTransform() method, with a retry mechanism.
    def wait_for_frames(self):
        retry_loop(self.listener.waitForTransform, func_args=[self.dest_frame, self.target_frame, rospy.Time.now(), self.timeout])
                
    ## Wrapper around lookupTransform() method, with a retry mechanism.
    #
    # @return the return of lookupTransform()
    def get_coordinates(self):
        ret, err = retry_loop(self.listener.lookupTransform, func_args=[self.dest_frame, self.target_frame, rospy.Time.now()])
        if ret == None:
            raise err
        return ret
        
    ## Calls self.get_coordinates() and extracts the (x,y) linear coordinates part of its result.
    #
    # @return an (x,y) tuple.
    def get_2d_coordinates(self): 
        linear_coords = self.get_coordinates()[0]
        return linear_coords[:2]
    

