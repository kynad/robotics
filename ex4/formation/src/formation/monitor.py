#!/usr/bin/python

from monitor_helper import *
import rospy, actionlib, my_utils
from formation.msg import RobotLocation


## @class Monitor
#  Implements the monitor node.
class Monitor:
    ##
    # @param number_of_robots - the size of the group to monitor (will be used to know for how many robots should this node wait, before start)
    # @param leader_name      - the string identifier of the robot that will get the leader role (e.g. "robot_2")
    def __init__(self, number_of_robots, leader_name):
        self.number_of_robots = number_of_robots
        self.leader_name = leader_name
        self.robots_group = GroupManager()        
        rospy.Subscriber(my_utils.C_MONITOR_TOPIC_NAME, RobotLocation, self.monitor_callback)
        rospy.loginfo("Working with %d robots, %s is the group leader." % (number_of_robots, self.leader_name))
        self.wait_for_robots()
        
    ## The callback function for incoming messages on the monitor topic.
    #  Upon recieving a robot's location, adds that robot to the group.
    #
    # @param data - the RobotLocation msg type.
    def monitor_callback(self, data):
       self.robots_group.add_robot(data.robot_name, (data.x_coord, data.y_coord))
       rospy.loginfo("Found robot %s, at (%f,%f)" % (data.robot_name, data.x_coord, data.y_coord))
          
    ## Waits for robots to send their locations.
    #  Ends waiting when the group is of the same size as the variable number_of_robots.
    def wait_for_robots(self):
        rate = rospy.Rate(my_utils.C_DEFAULT_RATE)
        while len(self.robots_group) < self.number_of_robots:
            rospy.logwarn("Still waiting for robots, only the following checked so far: %s " % str(self.robots_group))
            rate.sleep()
        rospy.loginfo("Robots are ready!")
        
    ## Tells the GroupManager object to calculate the actions and send them to the robots.
    def send_actions(self):
        self.robots_group.send_actions(self.leader_name)
        
    ## Tells the GroupManager to begin waiting for robots to finish the actions.
    def wait_for_results(self):
        self.robots_group.wait_for_results()


if __name__ == "__main__":
    rospy.init_node("monitor")
    rospy.sleep(1)
    group_size = rospy.get_param("number_of_robots")
    leader_name = rospy.get_param("leader_name")
    monitor = Monitor(group_size, leader_name)
    monitor.send_actions()
    monitor.wait_for_results()
    rospy.spin()
    
