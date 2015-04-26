#!/usr/bin/python

import rospy, my_utils, actionlib
from formation_robot import Leader, Follower
from formation.msg import RobotLocation, StartFollowAction

## @class Robot
#
#  A robot that is waiting for a goal message to perform one of the supported actions:
#   - Lead
#   - Follow (robot X)
#
class Robot:
    ##
    # @param name - the string identifier of the robot.
    def __init__(self, name):
        self.name = name
        self.publisher = rospy.Publisher(my_utils.C_MONITOR_TOPIC_NAME, RobotLocation, queue_size=10, latch=True)
        self.start_server()

    ## Starts the action server.            
    def start_server(self):
        rospy.loginfo("%s: starting server" % self.name)
        self.server = actionlib.SimpleActionServer(my_utils.C_ACTION_SERVER_STR % self.name, StartFollowAction, self.execute, False)
        self.server.start()
        rospy.loginfo("%s: server started" % self.name)

    ## Finds self location and sends them to the monitor node (which also servers as hello).
    def send_location(self):
        x_y = my_utils.TransformListenerWrapper("/map", "/%s/base_footprint" % self.name).get_2d_coordinates()
        status_msg = RobotLocation(None, self.name, *x_y)
        rospy.loginfo("%s: sending location." % self.name)
        self.publisher.publish(status_msg)
        
    ## The callback function that is called on each goal received.
    def execute(self, goal):
        if goal.local_leader == self.name:
            role = Leader(self.name, None)
        else:
            role = Follower(self.name, goal.local_leader)
        try:
            role.start()
        except rospy.ROSInterruptException: # The only reason for this is to avoid exceptions on shutdown.
            pass
        self.server.set_succeeded()    

        
if __name__ == "__main__":
    name = rospy.get_param("tf_prefix")
    rospy.init_node(name)
    rospy.sleep(1)
    robot = Robot(name)
    robot.send_location()
    rospy.spin()

